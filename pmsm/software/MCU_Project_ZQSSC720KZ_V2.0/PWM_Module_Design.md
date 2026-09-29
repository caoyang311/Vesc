# TIM1 PWM平台模块详细设计

## 1. 设计范围

本文档定义`Platform/Pwm`模块适配当前TIM1底层初始化后的接口和职责，覆盖：

1. TIM1 CH1~CH3及CH1N~CH3N六路互补PWM输出。
2. 每一路PWM输出对应的预装载值设置。
3. 六路互补PWM输出的统一启动和关闭。
4. TIM1 CH4作为ADC注入组转换触发源的启动和关闭。
5. TIM1 CH4预装载值设置，用于调整ADC采样窗口。

本文档作为`Platform/Pwm`模块的实现依据。底层TIM1、ADC和GPIO初始化继续由CubeMX生成的Core模块负责。

## 2. 当前底层配置事实

根据当前工程配置：

- PWM定时器句柄为`htim1`。
- TIM1使用中心对齐模式。
- CH1、CH2、CH3分别用于三相PWM主输出。
- CH1N、CH2N、CH3N分别用于三相PWM互补输出。
- CH4配置为无引脚输出的PWM通道，用于产生比较事件。
- ADC2和ADC3的注入组外部触发源均配置为`T1_CC4`。
- TIM1的计数周期由ARR决定，当前ARR为3599；实际预装载值必须满足`0 <= CCR <= ARR`。
- TIM1底层初始化、GPIO复用、死区和刹车保护不属于Pwm模块配置职责。

关键约束：同一PWM组的主输出和互补输出共享同一个CCR。Pwm模块可以独立设置CH1、CH2和CH3三组预装载值，但不能为CH1与CH1N设置两个不同CCR。

## 3. 分层位置

```text
Algorithm / FastControl
        |
        v
App / MotorControl
        |
        v
Platform / Pwm
        |
        v
Core / TIM1 HAL初始化
        |
        v
STM32 HAL / TIM1硬件
```

依赖规则：

- FOC算法只使用逻辑PWM通道和预装载值接口。
- App不访问`TIM_HandleTypeDef`、HAL通道宏、CCR寄存器或GPIO。
- `Pwm.h`不得包含`tim.h`和STM32 HAL头文件。
- `Pwm.c`可以包含`tim.h`，并负责将逻辑通道映射到HAL通道。
- Pwm模块不实现FOC计算、不读取ADC结果、不处理通信协议。
- Pwm模块不负责修改TIM1频率、中心对齐模式、死区、刹车输入和GPIO复用配置。

## 4. 接口设计

### 4.1 PWM逻辑通道

建议使用枚举表示三组PWM逻辑通道：

```c
typedef enum
{
    PWM_CHANNEL_U = 0,
    PWM_CHANNEL_V,
    PWM_CHANNEL_W,
    PWM_CHANNEL_COUNT
} Pwm_ChannelType;
```

一个逻辑通道代表一对互补输出：

| 逻辑通道 | 主输出 | 互补输出 | 预装载寄存器 |
|---|---|---|---|
| `PWM_CHANNEL_U` | TIM1_CH1 | TIM1_CH1N | CCR1 |
| `PWM_CHANNEL_V` | TIM1_CH2 | TIM1_CH2N | CCR2 |
| `PWM_CHANNEL_W` | TIM1_CH3 | TIM1_CH3N | CCR3 |

### 4.2 预装载值接口

```c
bool Pwm_SetCompare(Pwm_ChannelType channel, uint16_t preload);
```

行为要求：

- `channel`必须小于`PWM_CHANNEL_COUNT`。
- `preload`必须不大于当前TIM1 ARR。
- 接口只更新对应CCR预装载寄存器，不改变PWM启停状态。
- 预装载寄存器是否在更新事件生效由TIM1底层OC预装载配置决定。
- 接口成功只表示CCR写入成功，不表示输出波形已经在当前计数周期生效。
- 该接口应保持常数时间执行，适合FastControl调用。
- 设置值为0或ARR时必须有明确行为：分别对应理论0%和100%比较边界；实际有效输出仍受PWM模式和硬件死区限制。

不建议继续使用百分比接口作为底层主接口。百分比换算属于上层策略，FOC通常直接产生定时器计数域的比较值，因此底层使用预装载值可以避免重复换算和额外精度损失。

### 4.3 六路互补PWM启动接口

```c
bool Pwm_StartOutputs(void);
bool Pwm_StopOutputs(void);
```

`Pwm_StartOutputs()`按固定顺序启动：

```text
CH1 -> CH1N -> CH2 -> CH2N -> CH3 -> CH3N
```

`Pwm_StopOutputs()`按安全顺序关闭六路输出。推荐先关闭互补输出，再关闭主输出：

```text
CH1N -> CH2N -> CH3N -> CH1 -> CH2 -> CH3
```

启动和关闭要求：

- 只能操作已由`MX_TIM1_Init()`完成初始化的`htim1`。
- 任一路HAL操作失败，接口返回`false`。
- 启动失败时应执行一次关闭流程，避免部分通道保持开启。
- 关闭接口应是幂等的，已关闭通道再次关闭不应产生错误副作用。
- Pwm模块不在接口内部启动ADC。
- 电机故障、过流或紧急停机路径应优先使用硬件Break/BDTR保护；软件关闭接口不能替代硬件保护。
- 启动PWM前应保证CCR1~CCR3已经写入安全初值。

### 4.4 TIM1 CH4 ADC触发接口

```c
bool Pwm_StartAdcTrigger(void);
bool Pwm_StopAdcTrigger(void);
bool Pwm_SetAdcTriggerCompare(uint16_t preload);
```

接口职责：

- `Pwm_StartAdcTrigger()`使能TIM1 CH4比较事件产生ADC注入组触发。
- `Pwm_StopAdcTrigger()`关闭CH4触发事件，不修改CH4当前预装载值。
- `Pwm_SetAdcTriggerCompare()`设置CCR4预装载值，调整ADC采样窗口。

CH4触发约束：

- CH4不需要GPIO输出，Pwm模块不启动CH4的物理输出引脚。
- CH4的比较事件必须与ADC2、ADC3的`T1_CC4`触发选择一致。
- CCR4必须不大于TIM1 ARR。
- 修改CCR4只影响后续比较事件，具体生效点由预装载和TIM1更新事件决定。
- FOC调用时应在允许的PWM更新窗口内写入CCR4，避免采样点处于不确定的边界。
- CH4触发启动不等同于ADC启动。ADC校准、注入组启动、DMA/中断和结果读取由ADC模块负责。
- 如果底层TIM1将CH4配置为PWM模式，Platform实现应使用与配置模式一致的HAL启动接口；设计上优先封装为“比较事件触发”，不向上层暴露HAL的PWM/OC实现细节。

## 5. 推荐内部映射

Pwm.c内部维护固定映射表：

```text
Pwm_ChannelType       HAL主通道       HAL互补通道       CCR
PWM_CHANNEL_U         TIM_CHANNEL_1   TIM_CHANNEL_1N    CCR1
PWM_CHANNEL_V         TIM_CHANNEL_2   TIM_CHANNEL_2N    CCR2
PWM_CHANNEL_W         TIM_CHANNEL_3   TIM_CHANNEL_3N    CCR3
ADC触发               TIM_CHANNEL_4   无                CCR4
```

映射表必须通过`_Static_assert`检查数量。Pwm.h不暴露HAL通道类型。

## 6. 初始化和启停流程

Pwm模块不记录初始化、PWM输出使能或ADC触发使能等运行时状态。调用方负责保证接口调用顺序，Pwm模块按请求直接校验参数并执行对应HAL操作。

推荐接口流程：

```text
System_Init
  -> MX_TIM1_Init
  -> Pwm_SetCompare(U, safe_value)
  -> Pwm_SetCompare(V, safe_value)
  -> Pwm_SetCompare(W, safe_value)
  -> Pwm_SetAdcTriggerCompare(adc_sample_value)
  -> Pwm_StartOutputs
  -> Pwm_StartAdcTrigger
  -> FastControl_Enable
```

停止流程：

```text
FastControl_Disable
  -> Pwm_StopAdcTrigger
  -> Pwm_StopOutputs
```

如果系统要求PWM和ADC触发必须同步启停，可在上层提供组合编排函数；Pwm底层仍保留两个独立接口，便于诊断和调试。

## 7. 与FastControl/FOC的调用边界

建议快速控制周期由TIM1更新事件或ADC注入转换完成事件触发，不能依赖当前1~200 ms普通任务槽。

```text
TIM1 PWM更新事件
  -> ADC注入转换
  -> ADC转换完成ISR
  -> FastControl入口
  -> FOC计算
  -> Pwm_SetCompare(U, CCR1_new)
  -> Pwm_SetCompare(V, CCR2_new)
  -> Pwm_SetCompare(W, CCR3_new)
  -> Pwm_SetAdcTriggerCompare(CCR4_new)
```

FastControl约束：

- ISR入口只做必要的采样结果搬运和快速控制调度。
- Pwm设置接口必须是有界、非阻塞、无动态内存。
- 不在Pwm接口中等待更新事件、不关闭全局中断、不调用通信和诊断服务。
- 新比较值应写入CCR预装载寄存器，由定时器更新事件分别生效。
- 三个PWM比较值和CH4采样点不要求通过统一提交接口同步更新；调用方应在同一控制周期内按固定顺序完成各CCR写入。

## 8. 安全边界和异常行为

| 异常输入或状态 | 处理要求 |
|---|---|
| 通道枚举越界 | 不访问映射表，返回`false` |
| PWM预装载值大于ARR | 不写CCR，返回`false` |
| CH4预装载值大于ARR | 不写CCR4，返回`false` |
| TIM1尚未初始化 | 调用顺序不满足前置条件；由System初始化流程保证，不在Pwm模块记录状态 |
| 六路启动中途失败 | 关闭本次调用中已启动的通道并返回`false` |
| 重复启动 | 不记录使能状态，直接调用底层启动接口并返回HAL执行结果 |
| 重复关闭 | 不记录使能状态，直接调用底层关闭接口并返回HAL执行结果 |
| PWM紧急关闭 | 优先依赖硬件Break；软件接口作为辅助路径 |

ARR读取策略：

- 每次设置预装载值时读取`__HAL_TIM_GET_AUTORELOAD(&htim1)`，适合配置固定但可能变化的场景。
- 如果ARR运行时恒定，也可在Platform配置中定义经过审查的周期常量，避免运行时读取；两者只能选择一种，不能产生不一致。
- 当前设计建议读取实际ARR，避免Pwm模块与CubeMX配置脱节。

## 9. 与ADC模块的职责边界

Pwm模块只产生TIM1 CH4比较事件，ADC模块负责：

- ADC1/ADC2/ADC3校准和使能。
- 注入组通道、触发源和采样时间初始化。
- 注入转换完成中断或DMA处理。
- 读取ADC注入结果并提供给FastControl。
- 处理ADC过载、溢出和错误状态。

禁止Pwm模块直接包含ADC头文件或操作ADC句柄。

## 10. 建议文件调整

```text
Platform/
  Pwm.h/.c
  Pwm_Config.h/.c       （仅在需要将通道映射、ARR策略配置化时增加）

App/
  Test.h/.c             （测试模块改为调用预装载值接口）
  MotorControl.h/.c     （后续调用PWM和ADC业务接口）

System/
  System.c              （初始化顺序编排）
  TaskConfig.c          （不承载FOC快速控制）

Core/
  Inc/tim.h
  Src/tim.c
  Inc/adc.h
  Src/adc.c
```

现有`Platform/Pwm.h/.c`应保留为硬件抽象边界，但需要从“百分比占空比接口”调整为“CCR预装载值接口”，并增加PWM输出和ADC触发的独立启停接口。

## 11. 验证计划

实现前后应至少验证：

1. 读取TIM1 ARR并测试`0`、`ARR`、`ARR+1`边界。
2. 分别设置CCR1、CCR2、CCR3，确认三组输出互不影响。
3. 设置CCR4并使用示波器或定时器事件观察ADC触发位置。
4. 启动后确认CH1/CH1N、CH2/CH2N、CH3/CH3N均输出且具有死区。
5. 关闭PWM后确认六路输出全部进入安全关闭状态。
6. 关闭ADC触发后确认PWM仍可继续运行。
7. 模拟任一路HAL启动失败，确认已启动通道被回滚关闭。
8. 在ADC转换完成事件中验证采样结果与CCR4位置的对应关系。
9. 检查FastControl调用的WCET，确认PWM写入不成为超时瓶颈。
10. 使用IDE诊断、编译器高警告级别和MISRA工具检查所有变更代码。

## 12. 待确认事项

以下内容在正式改码前需要结合新的底层初始化确认：

- TIM1 CH4当前使用`HAL_TIM_PWM_Start()`还是`HAL_TIM_OC_Start()`能够正确使能`T1_CC4`触发事件；应以生成代码中的OC模式和寄存器配置为准。
- TIM1预装载值的类型是16位还是统一使用32位接口后在边界处检查；当前STM32F3 TIM1为16位计数器。
- ADC注入组由ADC2、ADC3同时触发时，采样窗口和转换完成中断的同步要求。
- PWM启动、ADC触发启动与ADC注入组启动的系统级先后顺序。
- 不增加`Pwm_CommitUpdate()`接口；三相CCR和CCR4由调用方在控制周期内分别写入，具体调用顺序由FastControl设计确定。
- 不新增硬件Break状态读取和恢复接口；故障保护由TIM1硬件Break机制和现有系统故障处理负责。
