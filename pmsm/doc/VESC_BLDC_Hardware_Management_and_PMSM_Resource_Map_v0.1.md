# VESC BLDC 多硬件管理与 PMSM 硬件资源映射

**版本**：v0.1  
**分析对象**：`d:/FOC/Vesc/bldc`  
**目标**：梳理 VESC BLDC 如何支持不同控制器硬件，并建立 PMSM 适配所需的硬件资源映射框架。

> 本文描述的是源码中的硬件管理机制和移植边界，不代表目标 PMSM 硬件已经完成适配。任何 PWM、ADC、栅极驱动和保护参数必须在无功率输出、限流和可观测条件下逐项验证。

---

## 1. 核心结论

VESC 并不是为每块硬件复制一套完整固件，而是采用：

```text
公共控制/通信/应用代码
              |
              v
      HW_SOURCE / HW_HEADER
              |
              v
       hw.h 统一硬件入口
              |
              v
     具体 hw_*.h / hw_*.c
              |
              v
  MCU、定时器、ADC、功率级和板级外设
```

不同硬件主要通过以下机制差异化：

1. **构建期选择硬件文件**：Makefile 根据板名生成 `HW_SOURCE` 和 `HW_HEADER` 编译宏。
2. **统一头文件入口**：`hwconf/hw.h` 包含当前目标硬件头文件，并提供默认宏和公共兼容接口。
3. **板级属性宏**：使用 `HW_HAS_*` 表示三电阻采样、相电阻采样、栅极驱动器、CAN、NRF 等能力。
4. **资源索引宏**：使用 `ADC_IND_*`、`HW_*`、`DRV*_*` 等宏描述 ADC、PWM、UART、SPI、编码器和保护资源。
5. **硬件动作宏**：使用 `ENABLE_GATE()`、`DISABLE_GATE()`、`IS_DRV_FAULT()`、`CURRENT_FILTER_ON()` 等把公共算法连接到板级 GPIO。
6. **硬件参数宏**：定义采样电阻、放大倍数、电压分压、温度曲线和 VESC Tool 配置限制。
7. **可选能力和默认实现**：`hw.h` 对未实现能力提供空操作或安全默认值，减少公共代码中的条件分支。
8. **公共模块只依赖统一接口**：电机控制模块尽量不直接依赖具体板名，而依赖 `hw.h` 暴露的资源宏和接口。

---

## 2. 构建期硬件选择链路

### 2.1 构建入口

主要文件：

- `Makefile`
- `make/fw.mk`
- `hwconf/hwconf.mk`
- 具体板级 `hw_*.h` 和 `hw_*.c`

构建逻辑可以抽象为：

```text
make fw_<board>
    -> 根据 hwconf 目录扫描可用 hw_*.h
    -> 解析目标板名
    -> 查找 hw_<board>.h
    -> 查找对应 hw_*_core.c，或回退到 hw_<board>.c
    -> 生成 -DHW_SOURCE="..." -DHW_HEADER="..."
    -> 编译公共源码和板级源码
```

`make/fw.mk` 将 `HW_SOURCE`、`HW_HEADER`、Git 信息和编译器信息作为编译宏传入。`conf_general.h` 会检查这两个宏是否存在；如果不存在则编译失败。

### 2.2 源码编译组成

`make/fw.mk` 将以下部分组合到一个固件目标：

| 类别 | 典型内容 | 是否与具体硬件相关 |
|---|---|---:|
| ChibiOS 启动、HAL、RTOS | 启动文件、ADC、PWM、CAN、SPI、线程 | MCU/平台相关 |
| 公共系统 | `main.c`、`irq_handlers.c`、`timeout.c`、`events.c` | 低相关，依赖硬件接口 |
| 电机控制 | `motor/*.c` | 算法公共，资源接口相关 |
| 通信 | `comm/*.c` | 协议公共，底层外设相关 |
| 编码器 | `encoder/*.c` | 驱动公共，GPIO/定时器/SPI 相关 |
| 板级 | `hwconf/board.c`、`hwconf/hw.c`、驱动器和 `shutdown.c` | 高相关 |
| 具体硬件配置 | `hw_*.h` | 最高相关 |

### 2.3 现有硬件目录组织方式

`hwconf` 下同时存在以下组织模式：

```text
hwconf/<厂商或产品>/<型号>/hw_<型号>.h
hwconf/<厂商或产品>/<型号>/hw_<系列>_core.h
hwconf/<厂商或产品>/<型号>/hw_<型号>.c
```

典型设计是：

- 顶层型号文件只声明型号变体，例如 `HW60_IS_MK6`；
- `*_core.h` 保存共用资源、ADC、限制和外设定义；
- 型号头文件通过宏选择不同 PCB 修订版；
- 只有确实需要板级运行时代码时才提供 `.c` 文件。

例如 `hw_example.h` 选择 `hw_example_core.h`，`hw_60_core.h` 根据 `HW60_IS_MK3`、`HW60_IS_MK4`、`HW60_IS_MK5`、`HW60_IS_MK6` 等宏选择硬件名称和差异资源。

---

## 3. 统一硬件入口 `hw.h`

[hw.h](file:///d:/FOC/Vesc/bldc/hwconf/hw.h) 是公共代码访问硬件能力的关键入口。

### 3.1 作用

1. 包含 `HW_HEADER` 指定的具体板级头文件；
2. 根据硬件属性包含对应的栅极驱动器头文件；
3. 检查 `HW_NAME` 等必要标识是否定义；
4. 为可选硬件功能提供默认空宏；
5. 提供公共 ADC、温度、输入电压和通信默认资源；
6. 提供硬件限制默认值和双电机资源回退规则。

### 3.2 硬件能力宏

常见能力宏包括：

| 宏 | 含义 | PMSM 适配意义 |
|---|---|---|
| `HW_HAS_3_SHUNTS` | 三个相电流采样电阻 | 可支持三相同步采样和电流重构 |
| `HW_HAS_PHASE_SHUNTS` | 电阻位于电机相线上 | 影响 V0/V7 采样策略和电流滤波 |
| `HW_HAS_PHASE_FILTERS` | 存在相电压滤波控制 | 需映射滤波开关 GPIO |
| `HW_HAS_DRV8301` | 使用 DRV8301 | 复用栅极驱动 SPI、故障和校准接口 |
| `HW_HAS_DRV8305` | 使用 DRV8305 | 同上 |
| `HW_HAS_DRV8316` | 使用 DRV8316 | 同上 |
| `HW_HAS_DRV8320S` | 使用 DRV8320S | 同上 |
| `HW_HAS_DRV8323S` | 使用 DRV8323S | 同上 |
| `HW_HAS_NO_CAN` | 无 CAN | 禁止或裁剪 CAN 功能 |
| `HW_HAS_PERMANENT_NRF` | 固定安装 NRF24 | 影响 SPI/引脚复用 |
| `HW_SHUTDOWN_HOLD_ON` | 支持电源保持/关机按键 | 影响启动、关断和备份存储 |
| `HW_HAS_DUAL_MOTORS` | 双电机资源 | PMSM 第一阶段建议不启用 |
| `INVERTED_SHUNT_POLARITY` | 电流采样极性反向 | 必须验证电流符号和 Park 变换方向 |

### 3.3 默认宏的意义

`hw.h` 对以下接口提供默认空操作或默认值：

- `ENABLE_GATE()`、`DISABLE_GATE()`；
- `DCCAL_ON()`、`DCCAL_OFF()`；
- `IS_DRV_FAULT()`；
- 电流/相电压滤波开关；
- 传感器端口 5 V/3.3 V 切换；
- MOSFET 温度和电机温度；
- Sin/Cos 编码器电压；
- ADC 电流读取宏。

这能使同一套公共源码支持不同硬件，但对 PMSM 移植不能简单依赖默认值。所有涉及功率级安全、采样和故障的宏都必须明确实现并完成测试。

---

## 4. 硬件资源分层

### 4.1 第 1 层：MCU 与启动资源

需要在板级文件和 ChibiOS 配置中确定：

- MCU 具体型号；
- 主频、外部晶振、内部 RC 和 PLL；
- Flash/RAM 布局；
- 启动文件和中断向量；
- FPU 和编译器 ABI；
- 看门狗、PVD、复位原因；
- ChibiOS `mcuconf.h`、`halconf.h`、`chconf.h`。

VESC 当前工程通过 `board.c` 的 `__early_init()` 执行硬件早期初始化，再进入 ChibiOS 时钟初始化。PMSM 适配时应保证功率级在复位期间处于关闭状态。

### 4.2 第 2 层：GPIO 与板级静态配置

`board.h`/`board.c` 负责：

- GPIO 默认模式；
- 上拉/下拉；
- 输出默认电平；
- 复用功能；
- GPIO 速度；
- 板级时钟和早期初始化。

需要特别确认：

- Gate Enable 默认是否为关闭；
- 驱动器 Fault 是否有正确上拉/下拉；
- ADC 引脚是否保持模拟输入；
- PWM 输出复位期间是否可能产生窄脉冲；
- 编码器和 SPI 片选默认电平是否安全。

### 4.3 第 3 层：三相 PWM 和功率级

板级文件需要描述：

- PWM 定时器；
- A/B/C 三相高低侧输出通道；
- 中心对齐或边沿对齐；
- PWM 频率；
- 定时器时钟和周期；
- 死区时间 `HW_DEAD_TIME_NSEC`；
- 主输出使能；
- 刹车/紧急关断；
- PWM 与 ADC 触发的同步关系；
- 双电机时的第二组 PWM 资源。

VESC 的 `main.c` 注释明确记录了 `TIM1`、`TIM2`、`TIM8` 等资源用途；`irq_handlers.c` 中 `TIM2` 采样比较中断调用 FOC 采样处理函数。因此 PMSM 适配必须同步记录：

```text
PWM 定时器 -> ADC 触发源 -> ADC 注入完成中断 -> FOC ISR -> PWM 更新
```

不能只填写 PWM 引脚，而忽略采样触发定时器和中断向量。

### 4.4 第 4 层：ADC 资源与索引

VESC 在板级头文件中定义 ADC 向量和逻辑索引，例如：

```text
ADC_IND_CURR1       相电流 1
ADC_IND_CURR2       相电流 2
ADC_IND_CURR3       相电流 3
ADC_IND_SENS1       相电压/辅助采样 1
ADC_IND_SENS2       相电压/辅助采样 2
ADC_IND_SENS3       相电压/辅助采样 3
ADC_IND_VIN_SENS    母线电压
ADC_IND_TEMP_MOS    功率管温度
ADC_IND_TEMP_MOTOR  电机温度
ADC_IND_VREFINT     内部参考电压
ADC_IND_EXT         外部输入 1
ADC_IND_EXT2        外部输入 2
```

同时定义：

- `HW_ADC_CHANNELS`：ADC 缓冲区总通道数；
- `HW_ADC_INJ_CHANNELS`：注入组通道数；
- `HW_ADC_NBR_CONV`：每组转换数量；
- ADC 采样序列；
- 触发源和 DMA；
- `GET_CURRENT1/2/3()`；
- `ADC_VOLTS(ch)`；
- `ADC_V_L1/L2/L3`；
- `ADC_V_ZERO`。

对于 PMSM，ADC 映射必须同时描述：

1. 物理 MCU ADC 通道；
2. ADC 序列位置；
3. 逻辑索引；
4. 采样时刻；
5. 电流正方向；
6. 零点偏置；
7. 放大器增益；
8. 采样电阻；
9. 可采样电流范围；
10. ADC 饱和和异常行为。

### 4.5 第 5 层：电流采样与换算

VESC 使用以下参数将 ADC 量转换为电流：

```text
电流 = ADC 电压 / (采样电阻 × 放大器增益)
```

关键参数包括：

- `CURRENT_SHUNT_RES`；
- `CURRENT_AMP_GAIN`；
- `CURRENT_CAL1/2/3`；
- `INVERTED_SHUNT_POLARITY`；
- `CURR1_DOUBLE_SAMPLE` 等；
- `HW_HAS_3_SHUNTS`；
- `HW_HAS_PHASE_SHUNTS`。

适配 PMSM 时要区分：

| 采样结构 | 对控制的影响 |
|---|---|
| 三相下桥臂分流 | 可按 PWM 窗口重构三相电流 |
| 三个相线分流 | 可在特定零矢量采样，需处理滤波和采样窗口 |
| 双分流 | 需要根据 PWM 扇区重构缺失相电流 |
| 单分流 | 采样窗口和重构约束更严格 |

### 4.6 第 6 层：母线电压和温度

典型换算宏：

```text
Vbus = ADC × V_REG / ADC_FULL_SCALE × (VIN_R1 + VIN_R2) / VIN_R2
```

温度则根据 NTC 电阻曲线和 Beta 参数计算。需要建立：

- `V_REG`；
- `VIN_R1`、`VIN_R2`；
- MOSFET NTC 位置和曲线；
- 电机 NTC 位置和曲线；
- 传感器断线/短路检测；
- 温度限值和降额策略。

### 4.7 第 7 层：栅极驱动器

VESC 将不同驱动器封装为独立模块：

- `drv8301.c/.h`；
- `drv8305.c/.h`；
- `drv8316.c/.h`；
- `drv8320s.c/.h`；
- `drv8323s.c/.h`。

公共硬件头文件通过 `HW_HAS_DRVxxxx` 选择驱动器，驱动器头文件进一步提供：

- 初始化；
- SPI 读写；
- 放大器增益设置；
- 栅极驱动器过流模式；
- DC 校准开关；
- Fault 读取和清除；
- `HW_RESET_DRV_FAULTS()`。

PMSM 适配必须确认：

- 驱动器使能电平；
- Fault 电平和锁存行为；
- SPI 模式、时序和片选；
- 栅极驱动电源欠压；
- MOSFET VDS 过流检测；
- 硬件过流响应时间；
- MCU 软件故障与驱动器硬件故障的映射。

### 4.8 第 8 层：位置传感器

板级资源通常定义：

- Hall 三路 GPIO；
- ABI 定时器和输入通道；
- SPI/SSC/BiSS-C 设备及片选；
- Sin/Cos ADC 通道；
- 旋变激励和采样接口；
- 编码器中断线；
- 编码器刷新定时器和 NVIC 向量。

公共 `encoder/encoder.c` 再依据电机配置选择具体驱动。PMSM 第一阶段建议只定义一种位置传感器，避免多个驱动器同时占用 SPI、定时器或 EXTI 资源。

### 4.9 第 9 层：通信和辅助外设

硬件头文件还会描述：

- CAN RX/TX GPIO、复用和 `CANDx`；
- UART 设备、TX/RX、复用和默认波特率；
- 永久 UART；
- SPI 设备、SCK/MOSI/MISO/NSS；
- I2C 设备、SCL/SDA；
- PPM/Servo ICU 定时器；
- 外部 ADC 输入；
- LED；
- IMU；
- NRF/LoRa；
- 关机/电源保持。

这些资源必须避免与 PWM、ADC、编码器和驱动器复用冲突。

---

## 5. 硬件限制如何进入 VESC Tool

VESC 通过板级宏给配置参数设定硬件边界，例如：

- `HW_LIM_CURRENT`；
- `HW_LIM_CURRENT_IN`；
- `HW_LIM_CURRENT_ABS`；
- `HW_LIM_VIN`；
- `HW_LIM_ERPM`；
- `HW_LIM_DUTY_MIN`；
- `HW_LIM_DUTY_MAX`；
- `HW_LIM_TEMP_FET`；
- `HW_LIM_FOC_CTRL_LOOP_FREQ`。

`commands_apply_mcconf_hw_limits()` 和 `commands_apply_appconf_hw_limits()` 会在配置交互过程中应用硬件限制。其作用是：

```text
VESC Tool 参数
    -> 固件接收
    -> 根据硬件宏限制范围
    -> 写入内部配置
    -> Flash 保存
    -> 控制算法使用
```

PMSM 适配时，限制值必须从以下因素推导，而不是照抄 VESC 板卡：

- DC 母线和电容耐压；
- MOSFET/IGBT 额定电流；
- 栅极驱动和采样放大器范围；
- 采样电阻热容量；
- PCB 铜厚和散热能力；
- 电机允许相电流；
- 控制环频率和 ADC 采样能力；
- 最高电角速度；
- 温度保护策略。

---

## 6. VESC 硬件接口与 PMSM 建议接口的对应关系

| VESC 机制 | PMSM 建议抽象 | 说明 |
|---|---|---|
| `ENABLE_GATE()` | `PmsmHw_GateEnable()` | 只负责允许功率级，不代表电机已经运行 |
| `DISABLE_GATE()` | `PmsmHw_GateDisable()` | 必须是确定性安全关断 |
| `IS_DRV_FAULT()` | `PmsmHw_GetGateDriverFault()` | 读取驱动器硬件故障 |
| `DCCAL_ON/OFF()` | `PmsmHw_SetCurrentAmpCalibration()` | 控制电流放大器零点校准 |
| `GET_CURRENT1/2/3()` | `PmsmHw_ReadPhaseCurrentAdc()` | 只提供原始采样，换算可放在采样层 |
| `ADC_IND_*` | `PmsmAdcChannelId` | 建议从宏升级为明确枚举/配置表 |
| `ADC_VOLTS()` | `PmsmAdc_ConvertToVoltage()` | 包含参考电压和 ADC 满量程 |
| `NTC_TEMP()` | `PmsmThermal_Convert()` | 统一温度传感器接口 |
| `READ_HALL1/2/3()` | `PmsmPosition_ReadHall()` | 统一位置采集接口 |
| `HW_ENC_*` | `PmsmPosition_Config` | 描述编码器定时器、GPIO 和中断 |
| `HW_CAN_*` | `PmsmCan_Config` | 描述 CAN 外设和引脚 |
| `HW_UART_*` | `PmsmUart_Config` | 描述 VESC Tool 备用通道或应用串口 |
| `HW_LIM_*` | `PmsmSafetyLimits` | 由硬件能力和安全分析确定 |
| `HW_EARLY_INIT()` | `PmsmHw_EarlyInit()` | 复位后先进入安全功率级状态 |
| `HW_SHUTDOWN_*` | `PmsmPower_Shutdown` | 产品需要电源保持时再实现 |

建议 VESC Tool 协议层和 PMSM 硬件层之间使用显式转换，不让 VESC 命令处理器直接调用 GPIO 或 ADC 寄存器。

---

## 7. PMSM 硬件资源映射表模板

以下表格是后续填写自有硬件的基线。

### 7.1 基本信息

| 项目 | 自有 PMSM 硬件 | 备注 |
|---|---|---|
| 控制器名称 | 待填写 | 产品/PCB 名称 |
| 硬件版本 | 待填写 | PCB 修订号 |
| MCU 型号 | 待填写 | 内核、主频、ADC 数量 |
| 编译器 | 待填写 | 工具链版本 |
| RTOS/HAL | 待填写 | 是否继续使用 ChibiOS |
| 母线额定电压 | 待填写 | V |
| 母线最大电压 | 待填写 | V |
| 连续相电流 | 待填写 | A |
| 峰值相电流 | 待填写 | A |
| PWM 频率 | 待填写 | Hz |
| FOC 控制频率 | 待填写 | Hz |
| 电机温度传感器 | 待填写 | NTC/PT1000/其他 |
| 位置传感器 | 待填写 | ABI/SPI/Hall/旋变/无感 |

### 7.2 功率级和 PWM

| 逻辑资源 | MCU 外设/通道 | GPIO/AF | 有效电平 | 备注 |
|---|---|---|---|---|
| PWM_A_H | 待填写 | 待填写 | 待填写 | 高侧 |
| PWM_A_L | 待填写 | 待填写 | 待填写 | 低侧 |
| PWM_B_H | 待填写 | 待填写 | 待填写 | 高侧 |
| PWM_B_L | 待填写 | 待填写 | 待填写 | 低侧 |
| PWM_C_H | 待填写 | 待填写 | 待填写 | 高侧 |
| PWM_C_L | 待填写 | 待填写 | 待填写 | 低侧 |
| PWM 主输出使能 | 待填写 | 待填写 | 待填写 | 定时器 BDTR/MOE 或外部使能 |
| Gate Enable | 待填写 | 待填写 | 待填写 | 栅极驱动使能 |
| Gate Fault | 待填写 | 待填写 | 待填写 | 硬件过流/驱动故障 |
| 紧急关断 | 待填写 | 待填写 | 待填写 | 硬件路径优先 |
| 死区时间 | 待填写 | 不适用 | 不适用 | ns |
| PWM 触发 ADC | 待填写 | 不适用 | 不适用 | TRGO/比较事件 |

### 7.3 ADC 采样

| 逻辑索引 | 物理信号 | MCU ADC 通道 | ADC 序列位置 | 采样组 | 换算/校准 |
|---|---|---|---|---|---|
| `ADC_IND_CURR1` | 相电流 A | 待填写 | 待填写 | 注入/规则 | 待填写 |
| `ADC_IND_CURR2` | 相电流 B | 待填写 | 待填写 | 注入/规则 | 待填写 |
| `ADC_IND_CURR3` | 相电流 C | 待填写 | 待填写 | 注入/规则 | 待填写 |
| `ADC_IND_VIN_SENS` | 母线电压 | 待填写 | 待填写 | 规则 | 分压比 |
| `ADC_IND_TEMP_MOS` | 功率器件温度 | 待填写 | 待填写 | 规则 | NTC 参数 |
| `ADC_IND_TEMP_MOTOR` | 电机温度 | 待填写 | 待填写 | 规则 | NTC 参数 |
| `ADC_IND_VREFINT` | 内部参考 | 待填写 | 待填写 | 规则 | VREF 校准 |
| `ADC_IND_EXT` | 外部输入 1 | 待填写 | 待填写 | 规则 | 可选 |
| `ADC_IND_EXT2` | 外部输入 2 | 待填写 | 待填写 | 规则 | 可选 |

必须单独记录：

| 项目 | 数值 |
|---|---|
| `HW_ADC_CHANNELS` | 待填写 |
| `HW_ADC_INJ_CHANNELS` | 待填写 |
| `HW_ADC_NBR_CONV` | 待填写 |
| ADC 分辨率 | 待填写 |
| ADC 参考电压 | 待填写 |
| ADC 采样周期 | 待填写 |
| PWM 到采样延迟 | 待填写 |
| ADC ISR 周期 | 待填写 |
| 电流零点 ADC 值 | 待填写 |
| 电流正方向 | 待填写 |

### 7.4 电流和电压换算

| 参数 | 符号/宏 | 数值 | 单位 |
|---|---|---:|---|
| ADC 参考电压 | `V_REG` | 待填写 | V |
| 相电流采样电阻 A | `CURRENT_SHUNT_RES_A` | 待填写 | Ω |
| 相电流采样电阻 B | `CURRENT_SHUNT_RES_B` | 待填写 | Ω |
| 相电流采样电阻 C | `CURRENT_SHUNT_RES_C` | 待填写 | Ω |
| 电流放大器增益 | `CURRENT_AMP_GAIN` | 待填写 | V/V |
| 母线分压上臂 | `VIN_R1` | 待填写 | Ω |
| 母线分压下臂 | `VIN_R2` | 待填写 | Ω |
| 母线最大允许电压 | `HW_LIM_VIN` | 待填写 | V |
| 电流 ADC 极性 | `INVERTED_SHUNT_POLARITY` | 待填写 | 是/否 |

### 7.5 位置传感器

| 项目 | 资源 | 数值 |
|---|---|---|
| 类型 | ABI/SPI/Hall/旋变/无感 | 待填写 |
| 接口外设 | TIM/SPI/ADC/EXTI | 待填写 |
| GPIO/AF | 待填写 | 待填写 |
| 片选/使能 | 待填写 | 待填写 |
| 编码器分辨率 | 待填写 | counts/rev |
| 电机极对数 | 待填写 | 对 |
| 机械零位偏移 | 待填写 | deg |
| 电角度方向 | 待填写 | 正/反 |
| 位置刷新频率 | 待填写 | Hz |
| 故障信号 | 待填写 | 断线/CRC/幅值/超时 |

### 7.6 通信和辅助资源

| 功能 | 外设 | RX/输入 | TX/输出 | 中断/DMA | 备注 |
|---|---|---|---|---|---|
| VESC Tool 通信 | USB/CAN/UART | 待填写 | 待填写 | 待填写 | 首选通道 |
| CAN | `CANDx` | 待填写 | 待填写 | 待填写 | 波特率 |
| 应用 UART | `SDx` | 待填写 | 待填写 | 待填写 | 可选 |
| 编码器 SPI | `SPIDx` | MISO | MOSI/SCK/NSS | 待填写 | 可选 |
| IMU | I2C/SPI | 待填写 | 待填写 | 待填写 | 可后置 |
| 关机输入 | GPIO/ADC | 待填写 | 待填写 | 待填写 | 可选 |
| LED | GPIO/PWM | 不适用 | 待填写 | 不适用 | 状态显示 |

---

## 8. 从 VESC 硬件文件移植到 PMSM 的顺序

### 阶段 1：只建立资源描述

完成：

- MCU、时钟和启动配置；
- GPIO 默认状态；
- PWM 六路输出资源；
- Gate Enable/Fault/紧急关断；
- ADC 物理通道和逻辑索引；
- 母线、电流、温度换算参数；
- 编码器接口资源；
- CAN/UART/USB 通道；
- 硬件安全限制。

此阶段不打开功率级，不输出 PWM。

### 阶段 2：验证低风险外设

按以下顺序：

1. MCU 时钟和复位原因；
2. GPIO 和 LED；
3. UART/USB；
4. CAN；
5. ADC 静态采样；
6. 温度和母线电压；
7. 编码器静态角度；
8. 栅极驱动器 SPI 读写；
9. Gate Fault 输入；
10. PWM 空载波形。

### 阶段 3：验证 PWM 和 ADC 同步

需要使用示波器验证：

- 三相 PWM 频率和中心对齐；
- 高低侧互补关系；
- 死区时间；
- Gate Enable 时序；
- ADC 触发点；
- ADC 注入转换完成中断；
- 电流采样窗口；
- 紧急关断响应。

### 阶段 4：接入 FOC

建议只启用：

- 有传感器位置；
- `Id = 0`；
- 受限 `Iq`；
- 低母线电压；
- 严格电流和占空比限制；
- 硬件过流保护。

### 阶段 5：接入 VESC Tool

最后接入：

- 硬件名称和版本；
- 实时数据；
- 电机参数；
- 硬件限制；
- 故障码；
- 参数保存。

---

## 9. 需要避免的移植方式

### 不建议 1：直接复制某块 VESC 的 `hw_*.h`

不同板卡的 ADC、PWM、分压、采样方向和保护电平可能完全不同。应复制结构，不复制未经验证的参数。

### 不建议 2：只修改 GPIO，不修改 ADC 时序

FOC 的正确性取决于 PWM、ADC 触发、采样窗口和 ISR 的整体时序。引脚对应正确并不意味着采样正确。

### 不建议 3：用软件限流替代硬件关断

软件过流检测不能替代驱动器或定时器的硬件刹车路径。硬件过流必须能够在 MCU 软件失效时关闭功率级。

### 不建议 4：让协议层直接操作硬件

VESC Tool 兼容层应调用 PMSM 控制服务；硬件资源应由 PMSM HAL/板级层提供。

### 不建议 5：一次启用所有可选功能

双电机、无感、HFI、IMU、LispBM、NRF 和自定义 UI 都会引入额外资源冲突和验证负担。第一版只保留最小 PMSM FOC 路径。

---

## 10. v0.1 结论和下一步

VESC 的多硬件支持本质上是：

```text
构建目标选择
    + 板级头文件资源描述
    + 公共硬件入口和默认宏
    + 栅极驱动器适配
    + ADC/PWM/编码器/通信资源映射
    + 硬件参数和 VESC Tool 限制
```

对于 PMSM 项目，建议形成一份独立的 `pmsm_hw_<board>.h/.c`，不要直接修改多个已有 VESC 板卡文件。该文件至少需要覆盖：

```text
MCU/时钟
GPIO 默认状态
三相 PWM 和死区
ADC 注入/规则采样
相电流和母线电压换算
温度换算
Gate Enable/Fault/紧急关断
位置传感器
CAN/USB/UART
硬件限制
故障和诊断资源
```

下一步应根据实际 PMSM 原理图和 MCU 引脚表填写本文第 7 节表格，然后再分析 `mcpwm_foc.c` 的具体资源使用和 ADC/PWM 时序。
