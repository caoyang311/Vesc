# VESC BLDC 源码架构地图

**版本**：v0.1  
**分析对象**：`d:/FOC/Vesc/bldc` 目录  
**参考源码版本**：当前工作区源码；`conf_general.h` 表示固件版本 `7.01`，ChibiOS 版本目录为 `ChibiOS_3.0.5`。  
**文档目的**：为 PMSM 硬件适配和 VESC Tool 兼容提供源码地图、模块边界、调用主线及移植优先级。

> 本文是源码结构分析基线，不等同于完整功能规格。具体功能行为仍需结合 `.c` 实现、配置结构体和 VESC Tool 协议定义逐项确认。

---

## 1. 总体架构

```text
+--------------------------------------------------------------+
|                    VESC Tool / 外部控制器                   |
+-------------------------------+------------------------------+
                                |
                    VESC 命令与数据包协议
                                |
+-------------------------------v------------------------------+
| comm/                                                             |
| packet | commands | USB/Serial | CAN | log                       |
+-------------------------------+------------------------------+
                                |
+-------------------------------v------------------------------+
| 系统服务与配置层                                                |
| main | conf_general | conf_custom | flash_helper | events       |
| timeout | terminal | bms | lispBM | qmlui                     |
+-------------------------------+------------------------------+
                                |
+-------------------------------v------------------------------+
| 应用输入与运行控制层                                            |
| applications/app.c                                               |
| ADC | PPM | UART | Nunchuk | PAS | NRF | 自定义应用           |
+-------------------------------+------------------------------+
                                |
+-------------------------------v------------------------------+
| 电机控制服务层                                                  |
| motor/mc_interface.c                                             |
| 状态、模式、限流、故障、统计、电机实例选择                       |
+-------------------------------+------------------------------+
                                |
+-------------------------------v------------------------------+
| 电机算法与实时执行层                                            |
| mcpwm.c | mcpwm_foc.c | foc_math.c | virtual_motor.c          |
| PWM/ADC ISR | Clarke/Park | 电流环 | 观测器 | SVPWM | 检测      |
+-------------------------------+------------------------------+
                                |
+-------------------------------v------------------------------+
| 硬件抽象与板级实现                                              |
| hwconf/hw.c | hw.h | board.c | gate driver | shutdown          |
| encoder/ | driver/ | ChibiOS HAL/RT                            |
+--------------------------------------------------------------+
```

### 关键架构结论

1. `motor/mc_interface.*` 是上层访问电机控制的主要门面，不建议应用和通信模块直接调用 `mcpwm_foc.*`。
2. `motor/mcpwm_foc.*` 是实时 FOC 执行核心，依赖硬件宏、ADC、PWM 定时器和编码器接口。
3. `hwconf/` 通过 `HW_SOURCE`、`HW_HEADER` 注入板级实现，构建系统可以为不同硬件选择不同板级文件。
4. `comm/commands.*` 负责 VESC 命令解析、参数传输和实时数据回复；`packet.*` 负责帧封装、CRC 和流式解码，两者应与 PMSM 控制算法解耦。
5. `datatypes.h`、`conf_general.h`、`mcconf_default.h` 和 `appconf_default.h` 共同构成共享数据模型和默认配置基础。
6. ChibiOS 同时提供 RTOS、HAL、线程、互斥锁、事件、定时器和外设驱动，是当前工程的基础平台。

---

## 2. 顶层目录职责

| 目录/文件 | 主要职责 | PMSM 移植判断 |
|---|---|---|
| `main.c`、`main.h` | 系统启动、线程创建、周期任务、初始化协调 | 必须重构初始化顺序，保留系统调度骨架 |
| `motor/` | BLDC/FOC 电机控制、控制接口、数学运算、虚拟电机 | 核心移植对象；优先迁移 FOC |
| `hwconf/` | 各种控制器板级配置、PWM/ADC/保护、栅极驱动 | 必须按自有硬件重新实现 |
| `encoder/` | ABI、SPI、SSC、PWM、SinCos、旋变等位置传感器 | 按自有传感器选择最小子集 |
| `comm/` | USB、串口、CAN、VESC 命令、数据包和日志 | VESC Tool 兼容核心 |
| `applications/` | PPM、ADC、UART、Nunchuk、PAS、NRF 和自定义输入 | 按产品需求选择，可后置 |
| `driver/` | EEPROM、软件 SPI/I2C、定时器、舵机、无线驱动等 | 仅迁移实际使用的驱动 |
| `imu/` | IMU 传感器、传输层、姿态融合和线程 | 非电机闭环必需，可后置 |
| `lispBM/` | LispBM 解释器及 VESC 扩展 | 高级脚本功能，可后置 |
| `qmlui/` | 自定义 VESC Tool QML UI | 需要定制上位机界面时再迁移 |
| `util/` | CRC、缓冲区、滤波器、数学、工作线程、内存池、LZO | 按依赖关系迁移 |
| `ChibiOS_3.0.5/` | RTOS 和 HAL 平台 | 若保留 ChibiOS，则作为基础依赖；否则建立等价 OSAL/HAL |
| `blackmagic/` | 固件内置调试探针功能 | 非最小 PMSM 控制必需 |
| `tests/` | 角度、序列化、包恢复、数学等测试 | 应优先保留并扩展 |
| `make/`、`Makefile` | 编译、板级选择、工具链和固件打包 | 需建立自有构建目标 |
| `documentation/` | CAN 等已有协议说明 | 作为协议适配参考 |

---

## 3. 启动与运行主线

```text
复位/启动文件
    -> ChibiOS 启动与 HAL 初始化
    -> main()
    -> 板级 GPIO、时钟、PWM、ADC、保护初始化
    -> conf_general_init()
    -> mc_interface_init()
    -> mcpwm/mcpwm_foc 初始化
    -> encoder_init()
    -> commands_init()
    -> comm_usb_init() / comm_can_init()
    -> app_init() / app_set_configuration()
    -> timeout、LED、周期线程启动
    -> 主循环或后台服务运行
```

### 运行时并发模型

| 执行上下文 | 典型职责 | 迁移关注点 |
|---|---|---|
| PWM/ADC 中断 | 电流采样、FOC 控制计算、PWM 更新、快速保护 | 最高优先级；必须先验证采样时序和执行时间 |
| 编码器中断/定时器 | 位置捕获、传感器刷新、错误检测 | 需匹配传感器接口和实时性 |
| CAN 接收/处理线程 | 接收帧、拆包、命令和状态处理 | 需设计与控制任务的并发边界 |
| USB 读/处理线程 | 接收字节流、解码 VESC 包、派发命令 | VESC Tool 基础连接路径 |
| 周期线程 | 转子角度上报、HSI 修调等低频任务 | 可按硬件需求裁剪 |
| 应用线程/回调 | 外部输入解析和目标值生成 | 只向 `mc_interface` 提供控制目标 |
| 故障/超时后台处理 | 输入超时、故障锁存、停止电机 | 安全状态机必须独立验证 |

---

## 4. 电机控制模块地图

### 4.1 控制入口

`motor/mc_interface.*` 对外提供：

- 电机实例选择，支持双电机架构
- `OFF`、`DETECTING`、`RUNNING`、`FULL_BRAKE` 等状态管理
- Duty、速度、位置、电流、刹车、手刹和开环目标设置
- 电机释放、立即刹车、输入忽略和超时处理
- 故障读取、故障停机和故障信息记录
- 母线电流、电机电流、转速、温度、能量、里程和统计数据
- ADC ISR 入口与电机控制定时器 ISR 入口

PMSM 适配时建议保留该层作为稳定的应用 API，并将内部实现改为 PMSM 控制服务。

### 4.2 FOC 核心

`motor/mcpwm_foc.*` 对外暴露的能力包括：

- FOC 初始化、配置和状态
- Duty、速度、位置、电流、刹车和开环控制
- `Id/Iq` 实际值、目标值和滤波值
- `Vd/Vq`、`Vα/Vβ`、调制量和占空比
- 编码器、Hall、观测器、反电动势相位
- 电流/电压偏置校准
- 电阻、电感、编码器偏移、Hall 表检测
- HFI、观测器和弱磁相关状态
- 定时器采样中断和 ADC 中断处理
- 音频/蜂鸣器调制等附加功能

典型数据流：

```text
ADC 注入采样
    -> 电流偏置补偿/电流重构
    -> Clarke 变换
    -> 位置角获取（编码器/Hall/观测器/HFI）
    -> Park 变换
    -> Id/Iq PI 电流控制
    -> 解耦、反电动势补偿、限幅/弱磁
    -> 反 Park 变换
    -> SVPWM/调制波
    -> 三相 PWM 更新
```

### 4.3 相关模块

- `foc_math.*`：FOC 数学运算和变换基础。
- `mcpwm.*`：传统 BLDC 六步换相/非 FOC PWM 路径及相关检测。
- `virtual_motor.*`：仿真或虚拟电机支持，适合测试和算法验证。
- `mcconf_default.h`：电机控制默认参数及配置模型。
- `datatypes.h`：电机类型、传感器模式、采样模式、观测器类型、故障码等枚举和共享类型。

---

## 5. 硬件抽象地图

### 5.1 板级选择机制

```text
Makefile
    -> 选择 PROJECT/board
    -> 定义 HW_SOURCE、HW_HEADER
    -> conf_general.h 检查并包含 hw.h
    -> hw.h 包含具体硬件头文件
    -> hw.c 包含具体硬件源文件
```

具体硬件头文件通常负责定义：

- MCU 和时钟相关宏
- 三相 PWM 定时器、通道、GPIO 和死区
- ADC 注入通道、采样顺序和 DMA
- 相电流、母线电压和温度换算参数
- Gate Enable、Gate Disable、驱动器故障和硬件急停
- CAN、编码器、Hall、LED、风扇和外部输入引脚
- 硬件电流、电压、温度限制

### 5.2 现有硬件相关模块

| 文件/目录 | 职责 |
|---|---|
| `hwconf/hw.h` | 统一硬件宏、驱动器包含和通用硬件接口 |
| `hwconf/hw.c` | 板级源文件包含、硬件 ID 计算等 |
| `hwconf/board.c/.h` | 板级初始化和板级定义 |
| `hwconf/drv*.c/.h` | DRV8301、DRV8305、DRV8316、DRV8320S、DRV8323S 等栅极驱动器 |
| `hwconf/shutdown.c/.h` | 关断/保护路径 |
| `driver/timer.*` | 通用定时器服务 |
| `ChibiOS_3.0.5/os/hal` | ADC、CAN、PWM、SPI、I2C、USB、串口等 HAL |
| `ChibiOS_3.0.5/os/rt` | 线程、调度、互斥锁、事件、消息和信号量 |

---

## 6. 位置传感器地图

`encoder/encoder.c` 是统一调度层，根据电机配置选择具体传感器，并负责初始化、刷新、定时任务和错误处理。

已发现的实现类别：

- ABI：`enc_abi.*`
- SPI 磁编码器：`enc_as504x.*`、`enc_mt6816.*`、`enc_mt6835.*`、`enc_ma782.*`、`enc_amt22.*`
- SSC/BiSS-C：`enc_tle5012.*`、`enc_bissc.*`
- 旋变：`enc_ad2s1205.*`
- SinCos：`enc_sincos.*`
- PWM 位置：`enc_pwm.*`
- 统一配置和类型：`encoder_cfg.*`、`encoder_datatype.h`

PMSM 第一阶段建议只实现一种传感器路径，优先顺序为：

1. 已明确接口和电角度方向的编码器；
2. 低速有传感器闭环 FOC；
3. 再增加无感观测器、Hall 或 HFI。

必须验证：机械角到电角换算、极对数、零位偏移、旋转方向、角度连续性和传感器故障行为。

---

## 7. 通信与 VESC Tool 兼容地图

### 7.1 数据包路径

```text
USB/CAN/UART 接收
    -> 字节流缓存
    -> packet_process_byte()
    -> 长度检查
    -> CRC16 校验
    -> commands_process_packet()
    -> 命令分发
    -> mc_interface / conf / app / encoder / bms
    -> commands_send_packet()
    -> packet_send_packet()
    -> USB/CAN/UART 发送
```

### 7.2 通信模块职责

| 模块 | 职责 | 兼容性重要度 |
|---|---|---:|
| `comm/packet.*` | 包头、长度、Payload、CRC16、结束标记和流式恢复 | 最高 |
| `comm/commands.*` | 命令枚举处理、配置读写、实时数据和回复 | 最高 |
| `comm/comm_usb.*` | USB CDC/串口读写线程及包处理 | 高 |
| `comm/comm_usb_serial.*` | USB 串行设备底层封装 | 高 |
| `comm/comm_can.*` | CAN 状态广播、CAN 命令、跨节点包转发 | 高 |
| `comm/log.*` | 日志和调试输出 | 中 |
| `documentation/comm_can.md` | CAN 通信说明 | 高，需逐条核对 |

### 7.3 VESC Tool 兼容分层

**最小兼容集：**

- 固件/硬件版本读取
- 实时数据读取
- 电机配置读取和写入
- 应用配置读取和写入
- 参数保存
- 故障读取
- 基础控制命令

**第二阶段：**

- CAN 配置和状态
- 电机检测
- 实验数据采样
- 自定义应用数据
- QML UI

**后置功能：**

- LispBM
- Blackmagic
- 完整固件升级和特殊产测功能

兼容策略：上层保持 VESC 的命令、序列化和数据包格式；内部建立 `VESC 兼容参数层 -> PMSM 内部参数层` 的转换，避免 VESC 数据模型与硬件驱动直接耦合。

---

## 8. 配置、参数与持久化

| 模块 | 作用 |
|---|---|
| `datatypes.h` | 枚举、故障码、通信和控制数据类型 |
| `conf_general.*` | 固件版本、硬件选择、编译开关、换算宏和全局配置初始化 |
| `mcconf_default.h` | 电机控制默认配置 |
| `appconf_default.h`、`applications/appconf_default.h` | 应用默认配置 |
| `conf_custom.*` | 自定义配置和 XML/QML 数据 |
| `confgenerator.*` | 配置生成/转换支持 |
| `flash_helper.*` | Flash 参数、固件辅助数据和完整性校验 |
| `eeprom.*` | EEPROM 或模拟 EEPROM 访问 |
| `crc.*`、`util/crc.*` | CRC 计算和数据完整性 |

PMSM 适配要求：

- 明确参数版本和结构体序列化规则；
- 对写入参数执行范围和 CRC 校验；
- 区分 VESC Tool 兼容字段与 PMSM 专用字段；
- 参数写入过程中禁止电机输出；
- 参数损坏时进入安全默认值或故障状态；
- 不让通信层直接操作硬件寄存器。

---

## 9. 功能清单与移植优先级

### P0：必须先完成

- ChibiOS/MCU 启动和基本调度
- GPIO、PWM、ADC、母线电压和温度采样
- Gate Enable/Disable 和硬件过流关断
- 电流偏置校准
- 一种位置传感器
- Clarke/Park、Id/Iq PI、SVPWM
- `mc_interface` 最小状态机
- 过流、过压、欠压、过温和传感器故障停机
- USB 或 CAN 基础通信
- VESC Tool 版本和实时数据读取

### P1：可用控制器功能

- 速度环、位置环和目标斜坡
- 再生制动和手刹
- 电机参数检测
- 参数 Flash 保存和恢复
- 完整故障记录和清除
- VESC Tool 电机/应用配置读写
- CAN 状态广播和节点控制
- 编码器零位检测和误差监视

### P2：扩展功能

- 无感观测器
- HFI 和低速无感启动
- 弱磁、MTPA 和高级解耦
- 双电机
- IMU、GNSS 和应用输入
- 数据采样、绘图和日志
- 自定义 QML UI
- LispBM
- Blackmagic
- 完整升级/产测功能

---

## 10. 推荐的源码阅读顺序

1. `Makefile`、`make/fw.mk`、`motor/motor.mk`：确定编译入口和模块依赖。
2. `main.c`、`irq_handlers.c`：建立启动、线程和中断时序图。
3. `hwconf/hw.h`、一个具体板级 `hw_*.h/.c`、`hwconf/hw.c`：建立硬件资源映射。
4. `mc_interface.c/.h`：理解上层控制 API、状态、故障和双电机选择。
5. `mcpwm_foc.c/.h`、`foc_math.c/.h`：分析实时 FOC 数据流和 ISR。
6. `encoder/encoder.c/.h` 与目标传感器驱动：确认角度来源和刷新周期。
7. `comm/packet.c/.h`、`commands.c/.h`：确认 VESC Tool 协议入口和参数序列化。
8. `comm/comm_usb.c`、`comm_can.c`：确认 USB/CAN 传输线程和包路由。
9. `conf_general.c`、`conf_custom.c`、`flash_helper.c`：确认配置生命周期。
10. `applications/app.c` 及输入模块：按产品需求逐项取舍。

---

## 11. PMSM 适配边界建议

```text
VESC Tool 协议/数据模型
        |
        v
VESC 兼容服务层（commands、packet、配置转换、故障映射）
        |
        v
PMSM 控制服务层（状态机、控制目标、保护、统计）
        |
        v
PMSM 算法层（FOC、观测器、SVPWM、参数检测）
        |
        v
PMSM HAL（PWM、ADC、位置传感器、保护、CAN、Flash）
        |
        v
MCU/驱动芯片/功率级/电机
```

以下内容不应在第一阶段直接混合：

- VESC Tool 命令解析与 PWM 寄存器操作；
- CAN 接收线程与 FOC ISR 共享复杂业务逻辑；
- 编码器具体协议与电机状态机；
- 参数序列化结构与板级 ADC 换算宏；
- 应用输入处理与功率级保护动作。

---

## 12. v0.1 结论

VESC BLDC 工程并不是单一的 BLDC 算法工程，而是由以下五类能力组成的完整固件平台：

1. **实时电机控制平台**：BLDC 六步、PMSM/BLDC FOC、检测、观测和控制环；
2. **多硬件板级平台**：通过 `hwconf` 和硬件宏适配多个 MCU/功率级；
3. **多传感器平台**：统一管理编码器、Hall、旋变和无感位置来源；
4. **设备通信平台**：通过 USB、串口和 CAN 提供 VESC Tool 兼容协议；
5. **应用与服务平台**：配置、Flash、故障、输入设备、IMU、脚本和调试扩展。

本项目的合理移植路径是：

```text
先建立 PMSM HAL
    -> 再建立安全的 PWM/ADC 采样链
    -> 再移植有传感器 FOC 电流环
    -> 再接入 mc_interface 状态机和保护
    -> 再实现 VESC Tool 最小协议
    -> 最后扩展速度/位置环、检测、CAN 和高级功能
```

**v0.1 下一步建议**：针对自有 PMSM 硬件建立《硬件资源映射表》和《PWM/ADC 采样时序表》，然后开始分析 `mcpwm_foc.c` 的 ISR 主流程。
