# MCU软件架构设计

## 1. 文档目的

本文档描述当前STM32F303 MCU工程已经实现的软件架构、模块边界、初始化顺序、任务模型、硬件触发链路及已知限制。

文档以当前参与构建的源码为准，并明确区分：

- **当前实现**：已经存在并接入工程的代码。
- **目标规划**：后续拟引入但当前尚未实现的模块。

当前工程采用轻量分层架构，不引入完整AUTOSAR Classic基础软件，但保留接口隔离、静态配置、职责分离和硬件抽象思想。

## 2. 设计原则

1. App负责业务行为，不直接操作寄存器。
2. System负责生命周期、时间服务和普通周期任务管理。
3. Platform提供DIO、PWM、ADC等硬件能力接口，并隔离HAL类型。
4. Core保留CubeMX生成的启动、外设初始化、中断入口和HAL句柄。
5. 普通毫秒级业务运行在主循环协作式任务域。
6. PWM同步采样和后续FOC运行在TIM1硬实时执行域。
7. ISR保持有界、非阻塞，不运行普通周期业务。
8. 使用静态内存，不使用动态任务注册和动态内存分配。
9. 公共头文件不向上层暴露HAL句柄、寄存器或物理引脚类型。
10. C代码遵循项目C Safety Coding Standard，采用MISRA C:2012和ISO 26262静态分析导向。

## 3. 当前总体架构

```text
+------------------------------------------------------------------+
| Core / Program Entry                                             |
| main.c / stm32f3xx_it.c / CubeMX MX_*_Init                      |
| - 芯片启动、HAL、系统时钟、外设初始化                            |
| - IRQ入口                                                        |
+-------------------------------|----------------------------------+
                                v
+------------------------------------------------------------------+
| System                                                           |
| System / SystemTime / TaskManager / TaskConfig                   |
| - 系统初始化编排                                                  |
| - 1 ms系统时间                                                    |
| - 静态协作式周期调度                                              |
+-------------------------------|----------------------------------+
                                v
+------------------------------------------------------------------+
| App                                                              |
| Led / Test                                                       |
| - LED业务                                                        |
| - DIO、PWM、ADC和CAN联调业务                                     |
+-------------------------------|----------------------------------+
                                v
+------------------------------------------------------------------+
| Platform                                                         |
| Dio / Pwm / AdcService / ServiceCallback                         |
| - 逻辑硬件接口                                                    |
| - ADC和定时器触发链路编排                                        |
| - HAL回调适配                                                     |
+-------------------------------|----------------------------------+
                                v
+------------------------------------------------------------------+
| Core Peripheral + STM32 HAL + CMSIS + Hardware                   |
| GPIO / DMA / ADC1-3 / TIM1、TIM4、TIM6 / CAN                    |
+------------------------------------------------------------------+
```

当前主要依赖：

```text
main -> System
System -> Led / Test / AdcService / TaskManager / SystemTime
TaskManager -> TaskConfig / SystemTime
TaskConfig -> Led / Test
Led -> Dio
Test -> Dio / Pwm / AdcService / CAN HAL（当前分层偏差）
AdcService -> Pwm / ADC HAL / TIM4 HAL
Pwm -> TIM1 HAL
Dio -> GPIO HAL
ServiceCallback -> SystemTime / AdcService / Test（当前跨层胶水）
```

## 4. 当前工程目录

```text
App/
  Led.h
  Led.c
  Test.h
  Test.c

System/
  System.h
  System.c
  SystemTime.h
  SystemTime.c
  TaskManager.h
  TaskManager.c
  TaskConfig.h
  TaskConfig.c

Platform/
  Dio.h
  Dio.c
  Dio_Config.h
  Pwm.h
  Pwm.c
  AdcService.h
  AdcService.c
  ServiceCallback.c

Core/Inc/
  main.h
  gpio.h
  dma.h
  adc.h
  tim.h
  can.h
  usart.h
  stm32f3xx_it.h

Core/Src/
  main.c
  gpio.c
  dma.c
  adc.c
  tim.c
  can.c
  usart.c
  stm32f3xx_it.c
  stm32f3xx_hal_msp.c
  system_stm32f3xx.c

Drivers/
  CMSIS/
  STM32F3xx_HAL_Driver/
```

目录职责：

| 目录 | 职责 |
|---|---|
| App | 业务状态、测试业务和功能任务 |
| System | 系统生命周期、时间和普通任务调度 |
| Platform | MCU硬件能力抽象及HAL回调适配 |
| Core | CubeMX生成的外设初始化、句柄和中断入口 |
| Drivers | STM32 HAL和CMSIS |

## 5. Core入口与底层初始化

### 5.1 main职责

`main()`执行底层初始化，随后只使用System生命周期接口：

```text
HAL_Init
SystemClock_Config
MX_GPIO_Init
MX_DMA_Init
MX_CAN_Init
MX_TIM1_Init
MX_ADC1_Init
MX_ADC2_Init
MX_ADC3_Init
MX_TIM2_Init
MX_TIM3_Init
MX_TIM4_Init
MX_USART1_UART_Init
MX_TIM6_Init
System_Init
while (1)
  -> System_MainFunction
```

`System_Init()`失败时进入`Error_Handler()`，不进入正常主循环。

### 5.2 Core边界

Core导出的`hcan`、`hadc1`、`hadc2`、`hadc3`、`htim1`、`htim4`和`htim6`均属于硬件相关接口。正常业务模块不应直接访问这些对象。

当前`Test`直接使用CAN HAL是已知临时偏差，后续应由CanIf替代。

## 6. System层

### 6.1 System

公共接口：

```c
bool System_Init(void);
void System_MainFunction(void);
```

当前初始化顺序：

```text
System_Init
  -> Led_Init
  -> SystemTime_Init
  -> Test_Init
       -> 设置CCR1、CCR2、CCR3和CCR4
       -> 启动六路互补PWM输出
       -> 启动CAN及FIFO0/错误通知
  -> AdcService_Calibrate(ADC1)
  -> AdcService_Calibrate(ADC2)
  -> AdcService_Calibrate(ADC3)
  -> AdcService_Start
       -> 启动ADC1循环DMA
       -> 启动ADC2注入组
       -> 启动ADC3注入组
       -> 启动TIM4 TRGO
       -> 启动TIM1 CH4 ADC触发
  -> TaskManager_Init
  -> SystemTime_Start
       -> 启动TIM1更新中断
       -> 启动TIM6 1 ms更新中断
```

初始化条件采用短路求值，任一步返回失败后不再执行后续步骤。

`System_MainFunction()`只调用`TaskManager_Run()`。

### 6.2 SystemTime

公共接口：

```c
void SystemTime_Init(void);
bool SystemTime_Start(void);
uint32_t SystemTime_GetTimeMs(void);
void SystemTime_Notification1ms(void);
```

实现特性：

- 使用`volatile uint32_t`保存毫秒计数。
- TIM6每1 ms通知一次，计数自然回绕。
- Cortex-M4上对齐的32位读取按单次原子访问使用。
- 当前`SystemTime_Start()`还启动TIM1更新中断，超出了纯时间服务职责。
- TIM1启动返回值当前被丢弃，接口返回值只反映TIM6启动结果。

### 6.3 TaskManager

公共接口：

```c
bool TaskManager_Init(void);
void TaskManager_Run(void);
```

任务配置结构：

```c
typedef void (*TaskManager_TaskFunction)(void);

typedef struct
{
    TaskManager_TaskFunction function;
    uint32_t period_ms;
    uint32_t start_delay_ms;
} TaskManager_TaskConfig;
```

调度特性：

- 静态任务表，无动态注册。
- 主循环协作式调度，无抢占和独立任务栈。
- 使用绝对截止时间。
- 每轮每个任务最多执行一次。
- 超期多个周期时采用常数时间取模推进，不连续补跑。
- 周期、首次延迟和时间判断限制在`uint32_t`半范围内。
- 通过纯无符号半范围规则处理系统时间回绕。
- 初始化时检查函数指针、非零周期和任务容量。

### 6.4 TaskConfig

当前固定周期槽：

| 周期槽 | 首次延迟 | 当前调用 |
|---:|---:|---|
| 1 ms | 0 ms | 空 |
| 5 ms | 0 ms | 空 |
| 10 ms | 0 ms | `Led_Task()`、`Test_Task()` |
| 20 ms | 0 ms | 空 |
| 50 ms | 0 ms | 空 |
| 100 ms | 0 ms | 空 |
| 200 ms | 0 ms | 空 |

最大任务槽数量为7，并通过编译期断言检查任务表容量。

新增普通业务应优先加入现有周期槽。FOC、电流环等硬实时控制不得加入该任务表。

## 7. App层

### 7.1 Led

公共接口：

```c
void Led_Init(void);
void Led_Task(void);
```

行为：

- 初始化时清零内部计数并将LED写低。
- `Led_Task()`由10 ms槽调用。
- 每调用50次翻转一次LED，即每500 ms翻转一次。
- 完整亮灭周期约1000 ms。
- 只使用`DIO_CHANNEL_LED`，不感知GPIOD和PD2。

调用链：

```text
TaskRun_10ms
  -> Led_Task
  -> Dio_FlipChannel(DIO_CHANNEL_LED)
  -> HAL_GPIO_ReadPin / HAL_GPIO_WritePin
```

### 7.2 Test

Test是当前硬件联调模块，不是最终产品业务模块。

主要职责：

- 设置三相PWM和ADC触发初始比较值。
- 启动六路互补PWM。
- 启动当前CAN测试链路。
- 每10 ms读取7路数字输入。
- 每10 ms读取ADC1温度信号及ADC2母线/转把信号。
- 收到标准ID `0x100`时，以ID `0x200`回发相同载荷。
- 提供运行时调整三相CCR和CCR4的测试接口。

当前测试初值：

| 对象 | 比较值 |
|---|---:|
| U相CCR1 | 900 |
| V相CCR2 | 1800 |
| W相CCR3 | 2700 |
| ADC触发CCR4 | 3595 |

Test当前直接包含`can.h`并调用HAL CAN，属于待CanIf替换的分层偏差。

## 8. Platform/Dio

### 8.1 公共接口

```c
Dio_LevelType Dio_ReadChannel(Dio_ChannelType ChannelId);
void Dio_WriteChannel(Dio_ChannelType ChannelId, Dio_LevelType Level);
Dio_LevelType Dio_FlipChannel(Dio_ChannelType ChannelId);
```

DIO逻辑通道使用枚举定义，公共头不暴露`GPIO_TypeDef`。

### 8.2 当前通道

| 逻辑通道 | 物理GPIO | 当前方向 | 用途 |
|---|---|---|---|
| `DIO_CHANNEL_LED` | PD2 | 输出 | LED |
| `DIO_CHANNEL_DSP_P` | PB9 | 输入 | P档 |
| `DIO_CHANNEL_DSP_SS` | PA3 | 输入 | 边撑 |
| `DIO_CHANNEL_DSP_SDL` | PC11 | 输入 | 低速档 |
| `DIO_CHANNEL_DSP_SDH` | PA15 | 输入 | 高速档 |
| `DIO_CHANNEL_DSP_FDS` | PB5 | 输入 | 防盗 |
| `DIO_CHANNEL_DSP_XH` | PC10 | 输入 | 定速巡航 |
| `DIO_CHANNEL_DSP_PUSH` | PB4 | 输入 | 推车 |
| `DIO_CHANNEL_DSP_SIF` | PA7 | 输出 | 一线通信 |

物理映射集中在`Dio_Config.h`及`Dio.c`配置数组中。

### 8.3 边界行为

- 无效Channel读取返回`STD_LOW`。
- 无效Channel写入不访问硬件。
- 无效Channel翻转返回`STD_LOW`。
- `STD_HIGH`写高，其他值写低。
- 当前接口未包含输入/输出方向权限检查。
- 当前输入由10 ms任务轮询，没有EXTI和软件消抖。

## 9. Platform/Pwm

### 9.1 公共接口

```c
bool Pwm_SetCompare(Pwm_ChannelType channel, uint16_t preload);
bool Pwm_StartOutputs(void);
bool Pwm_StopOutputs(void);
bool Pwm_StartAdcTrigger(void);
bool Pwm_StopAdcTrigger(void);
bool Pwm_SetAdcTriggerCompare(uint16_t preload);
```

接口直接设置CCR预装载值，不使用百分比换算，不保存初始化或运行状态。

### 9.2 通道映射

| 逻辑通道 | 主输出 | 互补输出 | 比较寄存器 |
|---|---|---|---|
| `PWM_CHANNEL_U` | TIM1 CH1 / PA8 | CH1N / PB13 | CCR1 |
| `PWM_CHANNEL_V` | TIM1 CH2 / PA9 | CH2N / PB14 | CCR2 |
| `PWM_CHANNEL_W` | TIM1 CH3 / PA10 | CH3N / PB15 | CCR3 |
| ADC触发 | TIM1 CH4，无GPIO输出 | 无 | CCR4 |

同一相的主输出与互补输出共享CCR，由硬件生成互补关系和死区，不能分别设置占空比。

### 9.3 TIM1当前配置

| 项目 | 当前配置 |
|---|---|
| 定时器时钟 | 72 MHz |
| Prescaler | 0 |
| 计数模式 | 中心对齐模式1 |
| ARR | 3599 |
| Repetition Counter | 1 |
| CH1/2/3 | PWM1 |
| CH4 | PWM2，作为ADC触发 |
| 死区值 | 100 |
| Break输入 | TIM1_BKIN，低有效 |
| PWM频率 | 约10 kHz |

比较值接口允许范围为0至ARR。功率级最小脉宽、死区安全裕量和ADC采样窗口由后续控制算法负责约束。

### 9.4 六路输出启停

`Pwm_StartOutputs()`依次启动CH1/CH1N、CH2/CH2N、CH3/CH3N。任一步失败时尝试停止全部输出。

`Pwm_StopOutputs()`执行普通软件关闭，不能代替硬件Break过流保护。

### 9.5 ADC触发

TIM1 CH4只产生ADC2和ADC3注入组触发事件，不输出到GPIO。CCR4用于调整PWM周期内的采样窗口。

当前不提供`Pwm_CommitUpdate()`，FastControl应在同一控制周期内按固定顺序写入CCR1、CCR2、CCR3和CCR4，由TIM1预装载机制在更新事件生效。

## 10. Platform/AdcService

### 10.1 公共接口

```c
bool AdcService_Calibrate(AdcService_UnitType unit);
bool AdcService_Start(void);
bool AdcService_ReadChannel(AdcService_UnitType unit,
                            AdcService_ChannelType channel,
                            AdcService_ValueType *value);
```

AdcService没有独立Config文件，当前ADC实例、通道、Rank和DMA映射固定在`AdcService.c`中。

### 10.2 校准与统一启动

System依次校准ADC1、ADC2和ADC3。AdcService不保存校准状态。

`AdcService_Start()`统一执行：

```text
HAL_ADC_Start_DMA(ADC1)
  -> HAL_ADCEx_InjectedStart(ADC2)
  -> HAL_ADCEx_InjectedStart(ADC3)
  -> HAL_TIM_Base_Start(TIM4)
  -> Pwm_StartAdcTrigger(TIM1 CH4)
```

任一步失败时尝试逆序停止TIM1 CH4、TIM4、ADC3、ADC2和ADC1 DMA。

### 10.3 ADC1规则组

公共配置：12位、右对齐、TIM4 TRGO上升沿触发、循环DMA。

| DMA索引 | 逻辑通道 | 物理通道 | 当前用途 |
|---:|---|---|---|
| 0 | `ADC1_IN6` | ADC1通道6 | 电机温度 |
| 1 | `ADC1_IN7` | ADC1通道7 | MOS U温度 |
| 2 | `ADC1_IN9` | ADC1通道9 | MOS V温度 |
| 3 | `ADC1_IN14` | ADC1通道14 | MOS W温度 |

TIM4配置为更新事件输出TRGO。当前约1 kHz触发ADC1规则序列，不需要启用TIM4更新中断：

```text
TIM4 Update Event
  -> TIM4 TRGO
  -> ADC1规则组转换
  -> DMA循环写入4个uint16_t
```

ADC1单通道读取为原子半字访问，但连续读取多个通道不保证属于完全相同的DMA序列快照。

### 10.4 ADC2注入组

| 注入Rank | 逻辑通道 | 当前用途 |
|---:|---|---|
| 1 | `ADC2_IN12` | U相电流 |
| 2 | `ADC2_IN8` | 转把电压 |
| 3 | `ADC2_IN5` | 母线电压 |

ADC2_IN2和ADC2_IN11为底层预留模拟输入，当前未加入转换序列，也未暴露逻辑读取接口。

### 10.5 ADC3注入组

| 注入Rank | 逻辑通道 | 当前用途 |
|---:|---|---|
| 1 | `ADC3_IN1` | V相电流 |
| 2 | `ADC3_IN12` | W相电流 |

ADC2和ADC3均由TIM1 CC4上升沿触发。FastControl路径直接读取JDR，不使用注入转换完成回调。

调用方必须保证读取时本周期注入转换已经完成；AdcService不轮询JEOC/JEOS，也不保存完成状态。

### 10.6 DMA HAL边界偏差

HAL ADC DMA接口固定使用`uint32_t *`，而当前DMA配置按半字传输至对齐的`volatile uint16_t`缓冲区。实现包含指针转换，需要按MISRA C:2012 Rule 11.8记录局部偏差并评审对齐、生命周期和DMA宽度依据。

## 11. 中断与快速控制域

### 11.1 TIM6系统时间

```text
TIM6每1 ms更新
  -> TIM6_DAC_IRQHandler
  -> HAL_TIM_IRQHandler
  -> HAL_TIM_PeriodElapsedCallback
  -> SystemTime_Notification1ms
  -> SystemTime_TickMs++
```

TIM6回调不运行TaskManager、Led或Test任务。

### 11.2 TIM1快速链

```text
TIM1中心对齐PWM
  -> CH4比较事件
  -> ADC2和ADC3注入转换
  -> 写入JDR

TIM1更新中断
  -> TIM1_UP_TIM16_IRQHandler
  -> HAL_TIM_PeriodElapsedCallback
  -> 读取U/V/W三相电流
  -> 预留FastControl/FOC入口
```

当前快速链只读取原始电流值，尚未实现：

- ADC偏置校正和物理量换算。
- 电流重构。
- FOC算法。
- 三相CCR闭环更新。
- 注入转换完成标志检查。
- 快速环超时和执行时间监控。

### 11.3 ISR规则

- 不调用阻塞函数和延时。
- 不运行普通任务槽。
- 不进行动态内存操作。
- 共享数据必须采用明确的单写者、快照、临界区或有界队列策略。
- ISR执行时间必须小于最短中断周期并纳入WCET评审。

## 12. 当前CAN实现

### 12.1 实现状态

当前只有测试级CAN链路，尚未实现CanIf、PduRouter、Com、CanTp和Diagnostic。

当前链路：

```text
Test_Init
  -> HAL_CAN_Start
  -> 激活FIFO0消息待处理和错误通知

CAN RX IRQ
  -> HAL_CAN_RxFifo0MsgPendingCallback
  -> HAL_CAN_GetRxMessage
  -> 写入单个全局message

TaskRun_10ms
  -> Test_Task
  -> 收到0x100
  -> 发送0x200相同载荷
```

当前只处理标准数据帧，DLC限制为1至8字节。

### 12.2 当前限制

- App/Test直接依赖`can.h`和HAL CAN。
- ServiceCallback直接写App全局消息，形成Platform到App的反向依赖。
- 接收只有单消息槽，连续报文会覆盖。
- ISR与主循环共享消息没有完整同步协议。
- 过滤器当前为全接收，软件再筛选标准帧。
- CAN错误回调为空。
- 没有发送确认、重试、Bus-Off恢复状态机和统计计数。
- 当前实现不具备正式通信服务的可扩展性和安全边界。

### 12.3 目标通信架构

后续采用轻量AUTOSAR职责划分：

```text
App
  -> Com / Diagnostic
Communication Services
  -> Com
  -> PduRouter
  -> CanTp
  -> Diagnostic
Platform
  -> CanIf
  -> CanIf_Config
  -> CanCallback
Core / HAL
  -> can.c / CAN IRQ / STM32 HAL CAN
```

接收分流：

```text
CAN IRQ
  -> CanIf有界Rx队列
  -> CanIf_MainFunctionRx
  -> PduRouter_RxIndication
       -> 普通通信PDU：Com
       -> 诊断PDU：CanTp -> Diagnostic
```

目标依赖：

```text
App -> Com或Diagnostic
Com -> PduRouter
Diagnostic -> CanTp -> PduRouter
PduRouter -> CanIf
CanIf -> HAL CAN
```

CanIf实施后应删除Test对HAL CAN的直接依赖，并将CAN HAL回调从当前综合`ServiceCallback.c`迁移到专用CanCallback边界。

## 13. 普通任务域与硬实时域

| 执行域 | 触发方式 | 当前内容 | 约束 |
|---|---|---|---|
| 普通周期任务域 | 主循环TaskManager | Led、Test | 非抢占、不得阻塞 |
| 系统时间ISR | TIM6更新中断 | 1 ms计数 | 短小、单写者 |
| 快速控制ISR | TIM1更新中断 | 读取三相电流、预留FOC | 硬实时、有界WCET |
| ADC1硬件链 | TIM4 TRGO + DMA | 温度等慢速模拟量 | 无TIM4更新中断 |
| CAN事件域 | CAN FIFO0中断 + 10 ms任务 | 单槽测试收发 | 后续改为ISR队列和主循环协议处理 |

普通任务必须满足：

- 不调用`HAL_Delay()`。
- 不忙等待外设完成。
- 不执行无限循环。
- 单次WCET明显小于任务周期。
- 不长时间关闭中断。

## 14. 模块依赖规则

| 模块 | 允许依赖 | 禁止依赖 |
|---|---|---|
| main | System、CubeMX初始化接口 | App业务细节、逻辑通道操作 |
| System | App初始化接口、System服务、Platform启动接口 | GPIO寄存器和ADC/PWM寄存器 |
| TaskManager | TaskConfig、SystemTime | App具体业务、HAL |
| TaskConfig | 任务公共接口 | HAL和物理配置 |
| Led | Dio | HAL、TaskManager、物理GPIO |
| Test | Dio、Pwm、AdcService；当前临时CAN HAL | 新增业务不应继续直接依赖HAL |
| Dio公共接口 | 标准整数类型 | HAL和GPIO类型 |
| Pwm公共接口 | 标准整数和布尔类型 | TIM句柄和HAL类型 |
| AdcService公共接口 | 标准整数和布尔类型 | ADC句柄、Rank宏和HAL类型 |
| Platform实现 | Core外设句柄和HAL | 不应依赖App业务状态 |
| ServiceCallback | 硬件事件通知接口 | 长业务、阻塞操作 |

## 15. 构建配置

顶层CMake当前加入：

```text
App/Led.c
App/Test.c
Platform/AdcService.c
Platform/Dio.c
Platform/Pwm.c
Platform/ServiceCallback.c
System/System.c
System/SystemTime.c
System/TaskManager.c
System/TaskConfig.c
```

用户模块include目录：

```text
App
Platform
System
```

CubeMX子目标提供Core、启动文件、HAL、CMSIS和链接配置。当前所有模块编译到同一固件目标中，分层依靠目录、公共头文件和依赖规则维持。

## 16. 可移植性边界

### 16.1 可直接保留

更换MCU平台时，以下逻辑可优先保留：

- TaskManager调度算法。
- TaskConfig周期槽组织方式。
- Led业务逻辑。
- Dio、Pwm和AdcService公共接口设计。
- 后续纯C FOC、PI和滤波算法。

### 16.2 需要适配

- `Dio.c`及GPIO映射。
- `Pwm.c`的互补输出、死区、Break和CCR实现。
- `AdcService.c`的ADC实例、Rank、DMA和触发实现。
- `SystemTime.c`和HAL回调适配。
- Core外设初始化和中断入口。
- CAN底层及未来CanIf实现。

## 17. 当前已知架构问题

按影响排序：

1. `Test_Init()`在ADC校准和采样链建立前启动六路PWM；后续初始化失败时System没有统一关闭PWM。
2. ADC2/ADC3由TIM1 CC4触发，但TIM1更新ISR读取JDR前不检查注入转换完成状态，依赖硬件时序假设。
3. `SystemTime_Start()`同时启动TIM1快速中断，职责混合，并忽略TIM1启动结果。
4. CAN ISR和10 ms任务共享单个消息对象，缺少有界队列和完整并发同步。
5. Test直接使用HAL CAN，ServiceCallback反向依赖App，违反目标分层。
6. CAN过滤、错误处理、Bus-Off恢复、发送确认和丢帧统计尚未完成。
7. ADC1多个DMA通道连续读取不保证同一序列快照。
8. DIO未限制通道方向，输入没有统一消抖和有效电平抽象。
9. PWM比较值只检查0至ARR，尚未加入功率级最小脉宽和采样窗口安全约束。
10. 缺少普通任务和快速控制ISR的WCET监控。

上述问题应独立评审和分阶段修复，不应通过跨层直接调用规避。

## 18. C Safety Coding Standard约束

- 公共头文件必须自包含且不泄漏HAL类型。
- 函数声明和定义必须使用Doxygen函数头；当前新增或修改注释使用中文。
- 使用固定宽度整数和`bool`。
- 输入指针、枚举、通道和数组索引在边界处校验。
- 所有`switch`包含`default`。
- 所有控制流使用花括号。
- HAL返回值必须检查，或通过显式`(void)`和注释记录有意丢弃。
- 不使用动态内存、递归、VLA和无界循环。
- ISR共享数据必须记录所有权和同步策略。
- MISRA偏差必须记录规则、位置、理由、风险和缓解措施。
- 正式发布前必须执行编译器高警告等级、MISRA C:2012静态分析和目标硬件测试。

## 19. 当前完整运行链

```text
初始化：
main
  -> CubeMX全部外设初始化
  -> System_Init
       -> Led_Init
       -> SystemTime_Init
       -> Test_Init
            -> 配置并启动六路PWM
            -> 启动CAN测试
       -> ADC1/2/3校准
       -> AdcService_Start
            -> ADC1 DMA + ADC2/3注入组
            -> TIM4 TRGO + TIM1 CH4
       -> TaskManager_Init
       -> SystemTime_Start
            -> TIM1快速中断 + TIM6时间中断

普通任务：
TIM6 1 ms -> SystemTime
main while -> TaskManager_Run
  -> 10 ms：Led_Task + Test_Task

慢速ADC：
TIM4 Update -> TRGO -> ADC1规则组 -> 循环DMA
  -> Test_Task读取温度信号

快速ADC：
TIM1 CH4 -> ADC2/ADC3注入转换
TIM1 Update IRQ -> 读取U/V/W电流 -> 预留FastControl

CAN测试：
CAN FIFO0 IRQ -> 单消息对象
10 ms Test_Task -> 0x100请求映射为0x200响应
```

## 20. 当前验收检查点

### 20.1 分层

- main只调用System生命周期接口。
- Led只通过Dio访问GPIO。
- Dio、Pwm和AdcService公共头不暴露HAL类型。
- TaskManager不感知具体业务模块。
- 当前CAN链分层偏差已明确，等待CanIf替换。

### 20.2 调度

- TIM6每1 ms推进系统时间。
- 任务表固定为1/5/10/20/50/100/200 ms周期槽。
- 到期任务每轮最多执行一次。
- 时间回绕使用无符号半范围判断。

### 20.3 功能

- LED每500 ms翻转一次，完整周期约1 s。
- TIM1输出三组互补PWM并提供CH4 ADC触发。
- ADC1由TIM4 TRGO触发并通过循环DMA更新4路结果。
- ADC2和ADC3由TIM1 CC4触发注入组转换。
- TIM1更新中断读取三相电流并保留FastControl入口。
- 当前CAN测试支持标准帧`0x100 -> 0x200`回环响应。

### 20.4 仍需硬件证据

- 三相PWM极性、死区和Break行为。
- TIM1 CH4实际触发边沿和ADC采样窗口。
- ADC2/ADC3转换在TIM1更新ISR前完成的时序裕量。
- ADC1四通道DMA顺序和1 kHz持续采样。
- 快速ISR和普通任务WCET。
- CAN高负载、错误帧和Bus-Off行为。
