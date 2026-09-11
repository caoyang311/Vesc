# PMSM STM32F407 第一版软件初始工程搭建指南

**版本**：v0.1  
**目标芯片**：STM32F407  
**工程定位**：基于 STM32CubeMX + STM32 HAL + CMake 的基础 PMSM 硬件工程  
**参考对象**：VESC BLDC 的硬件管理方式和 VESC Tool 兼容目标

---

## 1. 文档目标

本指南用于搭建第一版 PMSM 软件初始工程，不直接移植完整 VESC `mcpwm_foc.c`，而是先建立稳定的：

- STM32F407 启动和时钟工程；
- CubeMX 外设配置工程；
- CMake 构建工程；
- PMSM 硬件抽象层；
- PWM、ADC、栅极驱动和保护资源边界；
- UART/CAN 基础通信；
- 后续 VESC 控制算法和 VESC Tool 协议的接入基础。

第一版的核心原则：

```text
CubeMX 生成底层代码
    -> CMake 管理构建
    -> PMSM HAL 封装 STM32 HAL
    -> PMSM 控制层调用 PMSM HAL
    -> VESC 兼容层独立处理协议
```

---

## 2. 推荐技术路线

### 2.1 构建系统

推荐使用：

```text
CMake + Ninja
```

也可以使用：

```text
CMake + MinGW Makefiles
```

不建议将 CubeMX 生成的 Makefile 作为最终主构建系统。VESC 原工程采用 Makefile，但你的工程需要逐步加入：

- PMSM 专用模块；
- CubeMX 生成代码；
- 主机侧单元测试；
- Debug/Release/Test 多配置；
- 静态检查；
- 后续 VESC 代码的选择性迁移。

CMake 更适合管理这些目标。

### 2.2 底层软件

第一版建议使用：

- STM32 HAL：外设基础驱动；
- 必要时使用 LL：优化 PWM/ADC 高频路径；
- 裸机主循环：低复杂度任务调度；
- 中断：PWM、ADC、Break、编码器等实时事件；
- 暂不引入 FreeRTOS；
- ChibiOS 作为 VESC 原工程参考，不作为第一版底层依赖。

---

## 3. 推荐目录结构

```text
pmsm/
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/
│   ├── stm32cubemx.cmake
│   ├── compiler_options.cmake
│   └── warnings.cmake
├── cubemx/
│   ├── STM32F407_PMSM.ioc
│   ├── Core/
│   ├── Drivers/
│   ├── startup_stm32f407xx.s
│   └── STM32F407xxx_FLASH.ld
├── board/
│   └── stm32f407_pmsm/
│       ├── pmsm_board_config.h
│       ├── pmsm_board.h
│       ├── pmsm_board.c
│       └── pmsm_board_limits.h
├── hal/
│   ├── pmsm_hal_gpio.c
│   ├── pmsm_hal_gpio.h
│   ├── pmsm_hal_pwm.c
│   ├── pmsm_hal_pwm.h
│   ├── pmsm_hal_adc.c
│   ├── pmsm_hal_adc.h
│   ├── pmsm_hal_can.c
│   ├── pmsm_hal_can.h
│   ├── pmsm_hal_uart.c
│   └── pmsm_hal_uart.h
├── control/
│   ├── pmsm_control.c
│   ├── pmsm_control.h
│   ├── pmsm_state_machine.c
│   └── pmsm_state_machine.h
├── vesctool/
│   ├── packet_adapter.c
│   ├── packet_adapter.h
│   ├── commands_adapter.c
│   └── commands_adapter.h
├── app/
│   ├── pmsm_app.c
│   └── pmsm_app.h
├── tests/
└── doc/
```

模块边界：

| 模块 | 责任 | 不应负责 |
|---|---|---|
| `cubemx/` | CubeMX 生成的 MCU/HAL 初始化 | 电机控制业务 |
| `board/` | 板级资源、能力和限制 | VESC Tool 命令解析 |
| `hal/` | 封装 HAL 外设访问 | 控制策略和参数协议 |
| `control/` | PMSM 状态机和控制算法 | 直接操作 HAL 寄存器 |
| `vesctool/` | VESC 数据包、命令和参数兼容 | 直接操作 GPIO/PWM |
| `app/` | 产品应用和主循环 | 实现底层驱动 |
| `tests/` | 主机侧算法/协议测试 | 依赖真实功率级运行 |

---

## 4. STM32CubeMX 工程设置

### 4.1 芯片和工程

在 CubeMX 中：

1. 新建 STM32F407 工程；
2. 根据实际封装选择具体型号，例如 `STM32F407VGTx`；
3. 创建工程名，例如 `pmsm_f407`；
4. 生成 `.ioc` 文件到 `cubemx/`；
5. 保留 CubeMX 生成的 `Core/`、`Drivers/`、启动文件和链接脚本；
6. 后续最终编译由 CMake 执行。

CubeMX 的 Toolchain/IDE 选项可以选择 STM32CubeIDE。这里主要用于生成源码，不代表最终必须使用 CubeIDE 构建。

### 4.2 Code Generator 选项

建议设置：

```text
[x] Copy only the necessary library files
[x] Generate peripheral initialization as a pair of '.c/.h' files per peripheral
[x] Keep User Code when re-generating
[x] Backup previously generated files when re-generating
```

建议避免直接修改 CubeMX 生成区的非 User Code 内容。所有 PMSM 业务代码放在 `board/`、`hal/`、`control/` 和 `app/` 中。

### 4.3 HAL/LL 选择

第一版使用：

```text
GPIO：HAL
TIM：HAL
ADC：HAL
CAN：HAL
SPI：HAL
UART：HAL
DMA：HAL
```

后续在测量确认 FOC ISR 性能不足时，再针对以下路径使用 LL 或寄存器：

- PWM 占空比更新；
- ADC 快速读取；
- 定时器触发配置；
- Break 状态读取；
- 高频中断入口。

---

## 5. 时钟和系统时间配置

### 5.1 初始时钟目标

常见 STM32F407 初始配置：

```text
SYSCLK：168 MHz
AHB：168 MHz
APB1：42 MHz
APB2：84 MHz
```

STM32F4 定时器时钟需要特别确认：

```text
APB1 Timer Clock = 84 MHz
APB2 Timer Clock = 168 MHz
```

不要直接使用 APB 总线频率推导 PWM 频率。

### 5.2 必须确认的时钟项目

- 外部晶振频率；
- HSE、PLL 输入和倍频；
- Flash 等待周期；
- 电压缩放等级；
- FPU 配置；
- I-Cache/D-Cache 配置；
- 定时器实际输入时钟；
- ADC 时钟；
- CAN 时钟；
- UART 时钟；
- 看门狗时钟。

### 5.3 系统时间基准

第一版可以保留 CubeMX 默认 SysTick：

```text
SysTick -> HAL 时间基准
```

如果后续使用 FreeRTOS，或需要将 SysTick 留给其他用途，再改用 TIM6 等基础定时器。

建议避免一个定时器承担多个强实时职责：

```text
TIM1/TIM8：三相 PWM、Break、ADC 触发
TIM2/TIM3：编码器或捕获
TIM6：HAL 系统时基
TIM7：低频周期任务
```

具体分配必须以原理图、引脚复用和芯片参考手册为准。

---

## 6. PWM 配置

### 6.1 推荐定时器

优先选择：

```text
TIM1 或 TIM8
```

高级定时器支持：

- 三相互补 PWM；
- 死区时间；
- Break 输入；
- MOE 主输出使能；
- 定时器比较事件；
- ADC 触发同步。

### 6.2 CubeMX 初始配置

```text
计数模式：Center Aligned
通道：CH1、CH2、CH3
互补通道：CH1N、CH2N、CH3N
PWM 输出：Enable
互补输出：Enable
Break：Enable（如果硬件有故障关断信号）
Dead Time：根据功率级计算
```

建议初始参数：

```text
PWM 频率：10 kHz～20 kHz
初始最大占空比：0.05～0.10
```

第一版不要直接使用大占空比驱动电机。

### 6.3 PWM 必须验证的项目

- 三相频率；
- 中心对齐波形；
- 高低侧互补关系；
- 死区时间；
- PWM 复位状态；
- Gate Enable 时序；
- Break 触发后的输出状态；
- ADC 触发点；
- 占空比更新是否在安全时刻完成。

### 6.4 Break 硬件关断链路

推荐链路：

```text
硬件过流/驱动器 Fault
        ↓
TIM1_BKIN 或 TIM8_BKIN
        ↓
定时器硬件关闭 PWM
        ↓
软件中断记录故障
```

软件轮询 GPIO 不能替代硬件 Break 关断。

CubeMX 中需要确认：

- Break State；
- Break Polarity；
- Automatic Output；
- Lock Level；
- Break 输入滤波；
- NVIC 定时器中断；
- 驱动器 Fault 的有效电平。

---

## 7. ADC 配置

### 7.1 推荐采样方式

第一版推荐：

```text
ADC1/ADC2
    + Injected Group
    + PWM 定时器触发
    + ADC 注入转换完成中断
```

如果目标芯片资源或采样拓扑不适合，也可以使用：

```text
Regular Group + Timer Trigger + DMA
```

不建议最终 FOC 方案使用软件触发 ADC。

### 7.2 建议采样信号

至少规划：

```text
相电流 A
相电流 B
相电流 C 或辅助采样
母线电压
功率器件温度
电机温度
内部参考电压
```

三电阻采样时，注入组可以规划为：

```text
Rank 1：Phase Current A
Rank 2：Phase Current B
Rank 3：Phase Current C
```

双电阻采样时，必须在控制算法中实现第三相电流重构，并验证采样窗口是否足够。

### 7.3 PWM 与 ADC 同步链路

```text
PWM 定时器比较事件
        ↓
ADC 外部触发
        ↓
ADC 注入转换
        ↓
JEOC/Injected Conversion Complete
        ↓
FOC 快速处理
        ↓
更新下一周期 PWM
```

CubeMX 重点配置：

- ADC External Trigger；
- Trigger Edge；
- Injected Rank；
- Sampling Time；
- ADC 多模式；
- DMA 或中断；
- EOC/EOS 行为；
- ADC NVIC 优先级。

---

## 8. GPIO 和板级硬件管理

### 8.1 CubeMX User Label

建议使用清晰标签：

```text
GATE_EN
DRV_FAULT
PWM_A_H
PWM_A_L
PWM_B_H
PWM_B_L
PWM_C_H
PWM_C_L
CURRENT_FILTER_EN
ENCODER_CS
LED_STATUS
PRECHARGE_EN
EMERGENCY_STOP
```

### 8.2 PMSM 板级接口

不要让业务代码到处直接调用 CubeMX 生成的 GPIO 宏。建议集中封装：

```c
void PmsmHw_GateEnable(void);
void PmsmHw_GateDisable(void);
bool PmsmHw_IsGateDriverFault(void);
void PmsmHw_CurrentCalibrationEnable(void);
void PmsmHw_CurrentCalibrationDisable(void);
```

与 VESC 硬件宏的对应关系：

| VESC | PMSM |
|---|---|
| `ENABLE_GATE()` | `PmsmHw_GateEnable()` |
| `DISABLE_GATE()` | `PmsmHw_GateDisable()` |
| `IS_DRV_FAULT()` | `PmsmHw_IsGateDriverFault()` |
| `DCCAL_ON()` | `PmsmHw_CurrentCalibrationEnable()` |
| `DCCAL_OFF()` | `PmsmHw_CurrentCalibrationDisable()` |
| `CURRENT_FILTER_ON()` | `PmsmHw_CurrentFilterEnable()` |
| `CURRENT_FILTER_OFF()` | `PmsmHw_CurrentFilterDisable()` |

如果为了阶段性复用 VESC 算法需要宏适配，应将宏集中放在兼容层，不要散落在公共业务代码中。

---

## 9. CMake 工程管理

### 9.1 CMake 目标建议

```text
pmsm_f407
pmsm_f407_debug
pmsm_f407_release
pmsm_unit_tests
pmsm_math_tests
pmsm_protocol_tests
```

建议逻辑分层：

```cmake
add_library(cubemx_hal STATIC
    cubemx/Core/Src/main.c
    cubemx/Core/Src/stm32f4xx_it.c
    cubemx/Core/Src/stm32f4xx_hal_msp.c
    cubemx/Core/Src/system_stm32f4xx.c
)

add_library(pmsm_hal STATIC
    hal/pmsm_hal_gpio.c
    hal/pmsm_hal_pwm.c
    hal/pmsm_hal_adc.c
    hal/pmsm_hal_can.c
)

add_library(pmsm_control STATIC
    control/pmsm_control.c
    control/pmsm_state_machine.c
)

add_library(vesc_protocol STATIC
    vesctool/packet_adapter.c
    vesctool/commands_adapter.c
)

add_executable(pmsm_f407
    app/pmsm_app.c
)

target_link_libraries(pmsm_f407
    pmsm_control
    pmsm_hal
    cubemx_hal
    vesc_protocol
)
```

实际文件名可以根据 CubeMX 生成结果调整。

### 9.2 CMake 必须管理的内容

- MCU 编译选项；
- `STM32F407xx` 预处理宏；
- `USE_HAL_DRIVER`；
- 启动文件；
- 链接脚本；
- Include 路径；
- HAL 源文件；
- 浮点 ABI；
- Debug/Release 优化级别；
- map 文件和 bin/hex 输出；
- 栈和堆配置；
- 单元测试目标；
- 静态检查目标。

### 9.3 构建配置建议

```text
Debug：-Og -g3
Release：-Os 或 -O2
Test：主机编译，不链接 STM32 启动文件
```

FOC 实时性能确定前，不要盲目使用高优化级别。应通过 GPIO 翻转、DWT 或定时器捕获测量 ISR 执行时间。

---

## 10. 是否引入 FreeRTOS

第一版建议暂不引入 FreeRTOS：

```text
ADC/PWM ISR：实时控制
主循环：通信、故障和低频任务
SysTick：系统时基
```

主循环可以先保持简单：

```text
while (1)
    -> Pmsm_App_MainFunction()
    -> Pmsm_Comm_MainFunction()
    -> Pmsm_Fault_MainFunction()
```

等出现以下需求后再引入 RTOS：

- USB 复杂协议；
- CAN 多节点管理；
- 参数存储线程；
- 数据记录；
- IMU；
- LispBM；
- 多应用并行运行。

VESC 原工程依赖 ChibiOS 线程、互斥锁、事件和消息机制。若后续选择移植相关模块，应单独设计 OSAL，不要在第一版同时解决完整 ChibiOS API 兼容问题。

---

## 11. 第一版外设范围

建议只配置：

```text
RCC
SYS
GPIO
DMA
TIM1 或 TIM8
ADC1/ADC2
UART
CAN
SPI（位置传感器需要时）
IWDG
```

暂不配置：

```text
IMU
NRF
LoRa
SD 卡
LispBM
Blackmagic
双电机
复杂应用输入
自定义 QML UI
```

建议初始资源分配：

| 功能 | 推荐资源 |
|---|---|
| 三相互补 PWM | TIM1 或 TIM8 |
| PWM Break | TIM1_BKIN 或 TIM8_BKIN |
| ADC 触发 | PWM 定时器 TRGO/比较事件 |
| 相电流采样 | ADC1/ADC2 注入组 |
| 编码器 ABI | TIM2/TIM3，按引脚复用确定 |
| 磁编码器 | SPI 外设 |
| VESC Tool CAN | CAN1 |
| 调试串口 | USARTx |
| HAL 时基 | SysTick，后续可改 TIM6 |
| 看门狗 | IWDG |

如果 SPI 同时连接栅极驱动器和编码器，需要确认独立片选、SPI 模式、访问互斥和 DMA 资源。

---

## 12. PMSM 板级配置建议

可以建立 `pmsm_board_config.h`，集中描述：

```text
MCU 和时钟
PWM 定时器和通道
ADC 通道和采样序列
电流采样拓扑
母线电压分压
温度传感器
栅极驱动器
编码器
CAN/UART/SPI/I2C
安全限制
```

建议将 VESC 宏映射转换为明确配置数据：

```c
typedef struct
{
    uint32_t pwm_frequency_hz;
    uint32_t foc_frequency_hz;
    float adc_reference_voltage;
    float current_shunt_resistance_ohm;
    float current_amplifier_gain;
    float vbus_divider_upper_ohm;
    float vbus_divider_lower_ohm;
    float max_phase_current_a;
    float max_input_current_a;
    float max_bus_voltage_v;
    float min_bus_voltage_v;
    float max_fet_temperature_c;
} PmsmBoardConfig;
```

推荐限制层次：

```text
硬件绝对限制
    > 软件控制限制
    > VESC Tool 可配置限制
```

VESC Tool 发送的参数不能突破硬件绝对限制。

---

## 13. 第一版实施步骤

### Step 1：建立 CubeMX 基线

完成：

- `.ioc` 文件；
- STM32F407 芯片和封装；
- RCC、时钟、Flash 和电源配置；
- GPIO 默认安全状态；
- 启动文件和链接脚本。

验收：

- CMake 可识别源码；
- 固件可编译；
- MCU 可下载；
- 能够进入 `main()`。

### Step 2：建立 CMake 工程

完成：

- 工具链文件；
- CubeMX 源码库目标；
- PMSM HAL 目标；
- PMSM App 可执行目标；
- Debug/Release Preset；
- bin/hex/map 输出。

验收：

- CMake 配置成功；
- 编译无链接错误；
- 下载运行正常。

### Step 3：验证低风险外设

顺序：

1. LED；
2. UART 日志；
3. SysTick；
4. CAN 回环或收发；
5. ADC 静态采样；
6. 母线电压换算；
7. 温度换算；
8. 编码器静态角度。

### Step 4：验证栅极驱动

完成：

- Gate 默认关闭；
- Gate Enable/Disable；
- 驱动器 SPI 读写；
- Driver Fault 读取；
- 故障清除；
- 驱动器硬件过流配置。

未验证前不要接入高压母线。

### Step 5：验证 PWM

完成：

- 三相 PWM；
- 互补输出；
- 死区；
- Break；
- MOE；
- 安全关断；
- 占空比限制。

### Step 6：验证 PWM/ADC 同步

使用示波器验证：

- PWM 触发点；
- ADC 采样窗口；
- ADC 注入完成中断；
- ISR 执行时间；
- 电流零点；
- ADC 饱和行为。

### Step 7：接入最小 PMSM 控制框架

先只建立接口和状态机，不立即接入完整 FOC：

```text
INIT
  -> READY
  -> CALIBRATION
  -> PWM_TEST
  -> FAULT
  -> SHUTDOWN
```

所有异常必须进入安全停机路径。

---

## 14. 第一版验收标准

```text
[ ] CMake 配置成功
[ ] Debug 构建成功
[ ] Release 构建成功
[ ] STM32F407 正常启动
[ ] 复位后 Gate 保持关闭
[ ] UART 能输出版本和启动信息
[ ] LED 状态指示正常
[ ] ADC 静态采样值正确
[ ] 母线电压换算与实测一致
[ ] 温度采样换算合理
[ ] PWM 频率正确
[ ] 三相互补和死区正确
[ ] Break 能硬件关闭 PWM
[ ] Driver Fault 能被软件读取
[ ] ADC 与 PWM 触发关系正确
[ ] ADC ISR 执行时间已测量
[ ] CAN 基础收发正常
[ ] 编码器接口能读取静态角度
[ ] 未接入电机时不会输出危险占空比
```

---

## 15. 与 VESC BLDC 的兼容边界

第一版建议采用以下边界：

```text
CubeMX 生成代码
    不承载 VESC 控制算法

PMSM HAL
    封装 STM32 HAL 和板级资源

PMSM 控制层
    只调用 PMSM HAL，不直接访问 CubeMX GPIO 宏

VESC 兼容层
    处理 packet、commands、参数和实时数据

VESC Tool
    通过 USB/CAN/UART 访问兼容层
```

VESC 原有硬件宏可作为迁移参考：

```text
HW_SOURCE/HW_HEADER -> pmsm_board_config.h
ENABLE_GATE()       -> PmsmHw_GateEnable()
DISABLE_GATE()      -> PmsmHw_GateDisable()
IS_DRV_FAULT()      -> PmsmHw_IsGateDriverFault()
ADC_IND_*           -> PMSM ADC 逻辑通道
HW_LIM_*            -> PMSM 硬件限制
```

不要为了“兼容 VESC”而把 ChibiOS API、VESC 硬件宏和 STM32 HAL 调用混在同一层。

---

## 16. v0.1 结论

第一版应采用：

```text
STM32CubeMX：芯片、时钟、引脚、ADC、PWM、DMA、中断配置
CMake：整个工程的构建和测试管理
STM32 HAL：基础外设访问
PMSM HAL：硬件资源抽象
PMSM Control：控制算法和状态机
VESC 兼容层：VESC Tool 协议和参数适配
ChibiOS：只作为 VESC 原工程参考
```

最优先完成的 CubeMX 配置是：

```text
STM32F407
+ TIM1/TIM8 三相互补 PWM
+ ADC 注入采样
+ TIM Break 硬件关断
+ UART 调试
+ CAN 通信
+ Gate Enable/Fault GPIO
```

在完成上述基础验证前，不建议移植完整 `mcpwm_foc.c` 或直接接入高压电机系统。

**下一步**：根据实际 PMSM 原理图和 MCU 引脚分配，填写硬件资源映射表，并建立第一版 `pmsm_board_config.h` 与 CMake 工程。 
