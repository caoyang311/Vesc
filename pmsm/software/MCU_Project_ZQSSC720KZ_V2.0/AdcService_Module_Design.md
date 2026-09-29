# ADC采集服务模块详细设计

## 1. 设计目标

新增区别于CubeMX底层`adc.c/.h`的ADC平台服务模块，为App和FastControl提供统一的ADC校准、启动和结果读取接口。

本阶段采用固定实现，不设计独立配置模块，目标如下：

1. 支持当前ADC1、ADC2和ADC3三个固定硬件实例。
2. 统一启动ADC转换、DMA和对应定时器触发源。
3. 提供按ADC模块和逻辑通道读取原始采样值的接口。
4. 隔离ADC句柄、HAL通道、规则组Rank、注入组Rank、DMA和定时器细节。
5. 使用固定容量和有界分支，不使用动态内存。
6. 满足FastControl直接读取ADC2和ADC3注入结果的确定性要求。

本文档用于确认AdcService实施方案，本次仅更新设计，不修改代码。

## 2. 模块命名与文件

为避免与Core层CubeMX生成的`adc.c/.h`冲突，模块命名为`AdcService`。

```text
Platform/AdcService.h
Platform/AdcService.c
```

本阶段不创建以下文件：

```text
Platform/AdcService_Config.h
Platform/AdcService_Config.c
Platform/AdcServiceCallback.c
```

ADC实例、通道、Rank、DMA缓冲和触发源映射直接在`AdcService.c`中固定实现，减少配置表、跨文件私有类型和通用分发逻辑。

## 3. 分层和依赖

```text
App / FastControl
        |
        v
Platform / AdcService
        |
        +----> Platform / Pwm
        |        - TIM1 CH4 ADC触发启停
        |
        v
Core / adc.c / tim.c / dma.c
        |
        v
STM32 HAL / Hardware
```

依赖规则：

- `AdcService.h`只包含标准整数和布尔类型，不包含`adc.h`、`tim.h`或HAL头文件。
- `AdcService.c`可以包含`adc.h`、`tim.h`和`Pwm.h`。
- App和FastControl不能直接访问`hadc1`、`hadc2`、`hadc3`、`htim4`、DMA句柄或HAL API。
- AdcService负责ADC转换链路的启动编排，包括DMA和定时器触发源。
- System只调用AdcService公共接口，不直接启动TIM4、TIM1 CH4或ADC DMA。
- ADC分辨率、数据对齐、采样时间、GPIO、时钟、Rank和触发源仍由CubeMX底层初始化负责。

## 4. 当前底层配置

### 4.1 公共属性

三个ADC均为：

- 12位分辨率。
- 右对齐。
- 单端输入。
- 原始值范围为0至4095。

公共采样值类型使用：

```c
typedef uint16_t AdcService_ValueType;
```

AdcService只返回原始计数，不执行电压、电流或温度换算。

### 4.2 ADC1规则组

| 属性 | 当前配置 |
|---|---|
| 通道 | ADC1_IN6、ADC1_IN7 |
| Rank | IN6为Rank 1，IN7为Rank 2 |
| 触发源 | TIM4 TRGO更新事件，上升沿 |
| DMA | DMA1 Channel1 |
| DMA模式 | Circular |
| DMA数据宽度 | 半字，内存地址递增 |

固定DMA缓冲区：

```text
AdcService_Adc1DmaBuffer[0] <- ADC1_IN6
AdcService_Adc1DmaBuffer[1] <- ADC1_IN7
```

### 4.3 ADC2注入组

| 物理通道 | 注入Rank | 触发源 |
|---|---:|---|
| ADC2_IN12 | 1 | TIM1 CC4上升沿 |
| ADC2_IN8 | 2 | TIM1 CC4上升沿 |
| ADC2_IN5 | 3 | TIM1 CC4上升沿 |

ADC2_IN2和ADC2_IN11当前仅为预留模拟输入，没有加入转换序列，不提供读取接口。

### 4.4 ADC3注入组

| 物理通道 | 注入Rank | 触发源 |
|---|---:|---|
| ADC3_IN1 | 1 | TIM1 CC4上升沿 |
| ADC3_IN12 | 2 | TIM1 CC4上升沿 |

ADC2和ADC3共用TIM1 CH4触发事件。

## 5. 公共类型

### 5.1 ADC模块

```c
typedef enum
{
    ADC_SERVICE_UNIT_1 = 0,
    ADC_SERVICE_UNIT_2,
    ADC_SERVICE_UNIT_3,
    ADC_SERVICE_UNIT_COUNT
} AdcService_UnitType;
```

### 5.2 逻辑通道

```c
typedef enum
{
    ADC_SERVICE_CHANNEL_ADC1_IN6 = 0,
    ADC_SERVICE_CHANNEL_ADC1_IN7,
    ADC_SERVICE_CHANNEL_ADC2_IN12,
    ADC_SERVICE_CHANNEL_ADC2_IN8,
    ADC_SERVICE_CHANNEL_ADC2_IN5,
    ADC_SERVICE_CHANNEL_ADC3_IN1,
    ADC_SERVICE_CHANNEL_ADC3_IN12,
    ADC_SERVICE_CHANNEL_COUNT
} AdcService_ChannelType;
```

逻辑通道用于屏蔽物理通道号和Rank。因为本阶段不使用配置表，通道归属和Rank在`AdcService.c`的有界`switch`中直接映射。

## 6. 公共接口

### 6.1 校准接口

```c
bool AdcService_Calibrate(AdcService_UnitType unit);
```

职责：

- 校验`unit`范围。
- 按固定映射选择`hadc1`、`hadc2`或`hadc3`。
- 执行单端ADC校准。
- HAL校准成功返回`true`，否则返回`false`。
- 不保存校准状态。

调用约束：

- 由`System_Init()`在启动采样链路前依次校准ADC1、ADC2和ADC3。
- 校准可能等待硬件完成，只允许在系统初始化上下文调用。
- 禁止从FastControl或ISR调用。

### 6.2 统一启动接口

```c
bool AdcService_Start(void);
```

为避免ADC2和ADC3共用TIM1 CH4时出现“一个ADC尚未启动，触发源已经运行”的中间状态，启动接口不再按Unit分别启动，而是一次启动当前全部ADC采集链路。

固定启动顺序：

```text
1. HAL_ADC_Start_DMA(ADC1, AdcService_Adc1DmaBuffer, 2)
2. HAL_ADCEx_InjectedStart(ADC2)
3. HAL_ADCEx_InjectedStart(ADC3)
4. HAL_TIM_Base_Start(TIM4)
5. Pwm_StartAdcTrigger()
```

接口职责：

- 使能ADC1规则组循环DMA。
- 使能ADC2和ADC3注入组，等待TIM1 CH4触发。
- 启动TIM4，使其更新事件触发ADC1规则组。
- 调用Pwm模块启动TIM1 CH4 ADC触发。
- 任一步骤失败时返回`false`。
- 不保存启动状态。
- System不再分别调用TIM4、DMA或`Pwm_StartAdcTrigger()`。

失败回滚策略：

- 如果ADC1 DMA启动失败，立即返回`false`。
- 如果ADC2或ADC3启动失败，不启动任何定时器触发源，并尝试停止此前已启动的ADC转换。
- 如果TIM4启动失败，不启动TIM1 CH4，并尝试停止ADC1 DMA及ADC2/ADC3注入组。
- 如果TIM1 CH4启动失败，停止TIM4，并尝试停止全部ADC转换。
- 回滚操作的返回值可显式丢弃，因为主失败状态已经确定；实现中必须通过`(void)`标识有意丢弃。

接口前置条件：

- `MX_ADC1_Init()`、`MX_ADC2_Init()`、`MX_ADC3_Init()`、`MX_TIM4_Init()`和`MX_TIM1_Init()`已经完成。
- ADC1、ADC2和ADC3校准均已成功完成。
- TIM1 CH4比较预装载值已由Pwm模块设置为有效采样位置。

### 6.3 单通道读取接口

```c
bool AdcService_ReadChannel(AdcService_UnitType unit,
                            AdcService_ChannelType channel,
                            AdcService_ValueType *value);
```

行为：

- `value`不得为`NULL`。
- `unit`和`channel`必须在枚举范围内。
- `channel`必须属于指定`unit`。
- ADC1通道直接读取循环DMA缓冲区对应元素。
- ADC2和ADC3通道通过`HAL_ADCEx_InjectedGetValue()`读取对应JDR。
- 接口不启动转换、不轮询转换完成状态、不阻塞。
- 成功时写入`*value`并返回`true`。
- 参数或通道归属错误时返回`false`，且不修改`*value`。

FastControl读取ADC2和ADC3的前置条件：本控制周期的TIM1 CH4注入转换已经完成。AdcService不轮询完成标志，也不保存转换完成状态。

### 6.4 停止接口

当前需求不提供公共停止接口。后续出现低功耗、模式切换或故障停采样需求时，再统一设计ADC和触发源的逆序关闭接口。

## 7. AdcService.c内部设计

### 7.1 固定对象

```c
#define ADC_SERVICE_ADC1_DMA_LENGTH (2U)

static volatile uint16_t AdcService_Adc1DmaBuffer[
    ADC_SERVICE_ADC1_DMA_LENGTH];
```

不维护Unit配置表、Channel配置表、软件快照、启动状态、校准状态或错误计数。

### 7.2 固定分发方式

`AdcService_Calibrate()`使用`switch (unit)`选择ADC句柄。

`AdcService_ReadChannel()`先校验Unit和Channel，再使用`switch (channel)`执行固定映射：

| 逻辑通道 | 所属Unit | 数据来源 |
|---|---|---|
| ADC1_IN6 | ADC1 | DMA Buffer[0] |
| ADC1_IN7 | ADC1 | DMA Buffer[1] |
| ADC2_IN12 | ADC2 | Injected Rank 1 |
| ADC2_IN8 | ADC2 | Injected Rank 2 |
| ADC2_IN5 | ADC2 | Injected Rank 3 |
| ADC3_IN1 | ADC3 | Injected Rank 1 |
| ADC3_IN12 | ADC3 | Injected Rank 2 |

所有`switch`必须包含`default`分支。每个分支执行固定次数的操作，保证WCET有界。

### 7.3 DMA并发

- DMA异步写入ADC1缓冲区，因此缓冲区声明为`volatile uint16_t`。
- Cortex-M4对齐的16位读取是原子的。
- 单通道读取可以直接返回对应元素。
- 两次独立读取不保证来自同一个ADC1规则序列。
- 按当前需求不增加序列计数器、双缓冲或完整快照接口。

### 7.4 ADC2/ADC3直接读取

- FastControl在固定时序点调用读取接口。
- AdcService只读取指定Rank的注入数据寄存器。
- 不使用注入转换完成中断。
- 不创建`AdcServiceCallback.c`。
- 不在读取接口中检查或等待转换完成。

## 8. 初始化和启动顺序

System只负责调用公共接口，不直接操作DMA和定时器触发：

```text
System_Init
  -> AdcService_Calibrate(ADC1)
  -> AdcService_Calibrate(ADC2)
  -> AdcService_Calibrate(ADC3)
  -> Pwm_SetAdcTriggerCompare(sample_point)
  -> AdcService_Start
       -> 启动ADC1循环DMA
       -> 启动ADC2注入组
       -> 启动ADC3注入组
       -> 启动TIM4 TRGO
       -> 启动TIM1 CH4 ADC触发
```

说明：

- 校准仍由System编排，因为校准失败需要传播到系统初始化错误处理。
- DMA和定时器触发源全部封装在`AdcService_Start()`内部。
- System不得再直接调用`HAL_ADC_Start_DMA()`、`HAL_TIM_Base_Start(&htim4)`或`Pwm_StartAdcTrigger()`。
- FastControl只调用`AdcService_ReadChannel()`，不负责启动ADC或触发源。

## 9. 错误处理

| 场景 | 处理要求 |
|---|---|
| Unit越界 | 返回`false`，不访问HAL句柄 |
| Channel越界 | 返回`false`，不读取DMA或JDR |
| `value == NULL` | 返回`false` |
| Channel不属于Unit | 返回`false`，不修改输出值 |
| HAL校准失败 | 校准接口返回`false` |
| ADC/DMA启动失败 | 启动接口回滚已启动步骤并返回`false` |
| TIM4启动失败 | 不启动TIM1 CH4，回滚ADC启动并返回`false` |
| TIM1 CH4启动失败 | 停止TIM4，回滚ADC启动并返回`false` |
| ADC2/ADC3转换未完成便读取 | 调用前置条件违反，接口不轮询检测 |

当前不提供异步ADC/DMA错误查询，不保存内部错误计数。

## 10. 后续扩展原则

本阶段固定支持3个ADC和7个有效逻辑通道，不建立通用配置层。

增加通道时：

1. 在CubeMX中加入规则组或注入组并确定Rank。
2. 扩展`AdcService_ChannelType`。
3. 在读取接口的固定`switch`中增加Unit归属和数据源映射。
4. 必要时扩大固定DMA缓冲区。
5. 增加对应边界和硬件测试。

增加ADC实例时：

1. 在Core层生成新ADC底层初始化。
2. 扩展`AdcService_UnitType`。
3. 扩展校准、启动和读取的固定分支。
4. 更新统一启动顺序及失败回滚。

只有当ADC实例或通道数量显著增加、多个项目需要复用映射时，再评估恢复独立Config模块。

## 11. 文件职责

```text
Platform/AdcService.h
  - Unit、Channel、Value公共类型
  - Calibrate、Start和ReadChannel公共接口

Platform/AdcService.c
  - ADC1固定DMA缓冲区
  - ADC1/ADC2/ADC3校准
  - 三个ADC、DMA、TIM4和TIM1 CH4统一启动编排
  - 固定Channel到DMA元素或注入Rank的映射
  - 单通道原始值读取

Platform/Pwm.h/.c
  - TIM1 CH4比较预装载值
  - TIM1 CH4 ADC触发启停

Core/adc.c/.h
  - ADC1、ADC2、ADC3底层初始化

Core/tim.c/.h
  - TIM1和TIM4底层初始化

Core/dma.c/.h
  - ADC1 DMA底层初始化
```

## 12. C Safety Coding Standard要求

实施时必须满足：

- 所有函数声明和定义使用中文Doxygen函数头。
- 使用`uint16_t`保存12位ADC值，使用`bool`返回状态。
- 公共头文件自包含，不泄漏HAL类型。
- Unit、Channel和输出指针使用前完成边界检查。
- `switch`具有`default`分支，所有控制流使用花括号。
- 所有自动变量初始化后使用。
- 不使用动态内存、递归、VLA或无界循环。
- HAL返回值必须检查；回滚阶段有意丢弃的返回值显式转换为`void`。
- DMA共享缓冲的`volatile`属性、原子访问和读取时序假设必须记录。
- FastControl读取接口保持有界、非阻塞，不执行校准、启动或轮询。
- 正式集成前执行编译器高警告等级和MISRA C:2012静态检查。

## 13. 验证计划

### 13.1 接口边界

- 校准接口测试ADC1、ADC2、ADC3和越界Unit。
- 读取接口测试所有7个有效Channel。
- 测试越界Unit、越界Channel、Unit与Channel不匹配以及`value == NULL`。
- 验证错误返回时输出值不被修改。
- 验证原始值边界0和4095。

### 13.2 启动链路

- 验证`AdcService_Start()`启动ADC1循环DMA。
- 验证`AdcService_Start()`启动ADC2和ADC3注入组。
- 验证TIM4仅在ADC1 DMA启动成功后启动。
- 验证TIM1 CH4仅在ADC2和ADC3均启动成功后启动。
- 对每个HAL启动步骤注入失败，验证后续触发源不启动且已启动步骤被回滚。

### 13.3 硬件采样

- ADC1在TIM4更新事件下持续更新IN6和IN7。
- ADC2在TIM1 CH4触发下更新IN12、IN8和IN5。
- ADC3在TIM1 CH4触发下更新IN1和IN12。
- 调整TIM1 CCR4后验证ADC2和ADC3采样窗口变化。
- 验证FastControl在转换完成时序点读取到当前周期JDR结果。

### 13.4 时序和静态分析

- 测量`AdcService_ReadChannel()`在FastControl路径中的WCET。
- 确认TIM4和TIM1触发频率不超过ADC序列转换能力。
- 确认ADC2、ADC3共用TIM1 CC4时的采样和读取时序。
- 执行IDE诊断、真实编译及MISRA C:2012静态分析。

## 14. 已确认的实施决策

1. 不创建`AdcService_Config.h/.c`，当前映射固定在`AdcService.c`。
2. 不创建`AdcServiceCallback.c`。
3. ADC3注入组触发源为TIM1 CC4。
4. ADC1 DMA为Circular模式。
5. AdcService提供独立校准接口，System负责调用校准并处理失败。
6. `AdcService_Start()`统一启动三个ADC、ADC1 DMA、TIM4和TIM1 CH4触发。
7. System不直接启动DMA或定时器触发源。
8. ADC2和ADC3由FastControl直接读取注入数据寄存器。
9. 不增加完整三相电流快照接口。
