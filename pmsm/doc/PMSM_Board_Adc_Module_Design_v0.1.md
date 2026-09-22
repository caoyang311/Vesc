# PMSM Board ADC 模块设计方案 v0.1

## 1. 文档目的

本文档定义 PMSM 工程 `Board/Adc` 板级 ADC 模块的职责、软件分层、配置模型和扩展接口。设计参考 AUTOSAR Classic BSW ADC Driver 的功能边界，同时结合当前 STM32F407 PMSM 工程的 ADC1/ADC2、DMA、注入组和 TIM1 触发资源进行落地设计。

本文档目标：

- 将 MCU/HAL ADC 细节封装在 `Board/Adc` 内；
- 支持 ADC 通道、ADC 组、触发源和数据传输方式的独立扩展；
- 区分规则组、注入组、软件触发和硬件触发；
- 为后续电流、电压、温度采样及 FOC 同步采样提供稳定接口；
- 避免业务层直接访问 `hadc1`、`hadc2`、DMA 和 ADC 寄存器。

## 2. 当前工程基线

当前工程已经由 CubeMX 生成底层 ADC 配置，主要资源如下：

| 资源          | 当前配置                                        | 设计结论                                  |
| ------------- | ----------------------------------------------- | ----------------------------------------- |
| ADC1          | 规则组 2 个转换，ADC_CHANNEL_0 和 ADC_CHANNEL_9 | 适合作为慢速或周期性规则采样组            |
| ADC1 DMA      | DMA2 Stream0，循环模式，半字传输                | 应由 ADC 模块封装启动、停止和结果缓冲管理 |
| ADC2 注入组   | ADC_CHANNEL_3/6/8，共 3 个转换                  | 适合作为 PMSM 电流同步采样组              |
| ADC2 注入触发 | TIM1_CH4 上升沿                                 | 应由 ADC 模块提供触发组配置和状态接口     |
| ADC 中断      | ADC_IRQn 已使能                                 | 应统一转发到 ADC 模块通知函数             |
| 现有 HAL 层   | `MX_ADC1_Init()`、`MX_ADC2_Init()`          | 保留为 MCU 初始化层，不作为业务接口       |

当前 ADC 具体配置证据：

- [adc.c](../pmsm/software/PMSM_FOC/Core/Src/adc.c)：ADC1 规则组、ADC1 DMA、ADC2 规则组和注入组；
- [adc.h](../pmsm/software/PMSM_FOC/Core/Inc/adc.h)：HAL ADC 句柄和初始化函数声明；
- [Pwm.c](../pmsm/software/PMSM_FOC/Board/Pwm/Pwm.c)：TIM1_CH4 ADC 触发输出控制。

说明：设计文档按 `Board/Adc` 作为目标模块目录编写。当前工程检索结果未发现已纳入构建的 `Board/Adc` 源文件，因此本文档定义的是后续模块实现基线，而不是对尚未提交接口的确认。

## 3. AUTOSAR ADC 功能映射

| AUTOSAR ADC Driver 能力             | Board/Adc 对应能力                     | 当前阶段   |
| ----------------------------------- | -------------------------------------- | ---------- |
| Adc_Init                            | `Adc_Init()`，加载静态配置并复位状态 | 必须支持   |
| Adc_SetupResultBuffer               | 为 ADC 组绑定结果缓冲区                | 必须支持   |
| Adc_StartGroupConversion            | 启动指定 ADC 组转换                    | 必须支持   |
| Adc_StopGroupConversion             | 停止指定 ADC 组转换                    | 必须支持   |
| Adc_ReadGroup                       | 读取最近一次完整组结果                 | 必须支持   |
| Adc_GetGroupStatus                  | 查询组状态                             | 必须支持   |
| Adc_Enable/DisableGroupNotification | 使能或关闭组完成通知                   | 建议支持   |
| Adc_Enable/DisableHardwareTrigger   | 控制硬件触发组                         | 建议支持   |
| Adc_GetStreamLastPointer            | 获取 DMA 流式缓冲当前位置              | DMA 组需要 |
| Adc_GetVersionInfo                  | 获取模块版本                           | 可选       |
| Adc_DeInit                          | 关闭 ADC、DMA 和通知                   | 建议支持   |

不建议在板级模块中直接复制完整 AUTOSAR 配置复杂度。应保留 AUTOSAR 的概念边界，但使用适合本项目的静态 C 配置和轻量接口。

## 4. 推荐软件分层

```text
应用层 / FOC / Driver_Test
            |
            v
Board/Adc/Adc.h、Adc.c
            |
            +-- ADC 组管理
            +-- 结果缓冲管理
            +-- 触发源管理
            +-- DMA 管理
            +-- 中断通知管理
            +-- 状态和错误管理
            |
            v
STM32 HAL / Core/Src/adc.c / DMA / TIM1
```

### 4.1 Board/Adc 的职责

- 管理 ADC 实例和 ADC 组；
- 管理规则组与注入组；
- 管理软件触发、外部硬件触发和连续转换；
- 管理 DMA 缓冲区和组结果一致性；
- 提供原始 ADC 计数读取接口；
- 提供组完成、超时、溢出和错误状态；
- 向上层发布转换完成通知；
- 屏蔽 HAL 句柄和底层寄存器。

### 4.2 Board/Adc 不负责的内容

- 不负责电压、电流、温度的物理量换算；
- 不负责 FOC 控制算法；
- 不负责传感器线性化和标定策略；
- 不直接决定功率级是否允许运行；
- 不在 ADC 驱动内部实现业务故障策略。

物理量换算应放在独立的信号处理或传感器服务模块中，故障策略由上层安全管理模块决定。

## 5. 可扩展配置模型

建议使用静态配置表描述 ADC 通道和 ADC 组，不在业务代码中写死通道号。

### 5.1 通道配置

每个通道至少应包含：

| 字段             | 说明                                         |
| ---------------- | -------------------------------------------- |
| `ChannelId`    | Board/Adc 逻辑通道 ID，不直接暴露 MCU 通道号 |
| `Instance`     | ADC1、ADC2 等 ADC 实例                       |
| `HwChannel`    | STM32 ADC_CHANNEL_x                          |
| `GroupId`      | 所属 ADC 组                                  |
| `Rank`         | 组内转换顺序                                 |
| `SamplingTime` | 采样周期配置                                 |
| `InputType`    | 普通模拟输入、相电流、母线电压、温度等       |
| `Enabled`      | 是否纳入当前构建配置                         |

逻辑通道示例：

```text
ADC_CHANNEL_ID_PHASE_U_CURRENT
ADC_CHANNEL_ID_PHASE_V_CURRENT
ADC_CHANNEL_ID_PHASE_W_CURRENT
ADC_CHANNEL_ID_DC_BUS_VOLTAGE
ADC_CHANNEL_ID_POWER_STAGE_TEMPERATURE
ADC_CHANNEL_ID_AUXILIARY_0
```

新增 ADC 通道时，只需增加逻辑通道枚举和配置表项，并更新对应组的 Rank 与结果缓冲长度，避免修改公共采样接口。

### 5.2 组配置

每个 ADC 组至少应包含：

| 字段                | 说明                             |
| ------------------- | -------------------------------- |
| `GroupId`         | ADC 组逻辑 ID                    |
| `Instance`        | 所属 ADC 实例                    |
| `GroupType`       | 规则组或注入组                   |
| `ConversionCount` | 组内转换数量                     |
| `TriggerSource`   | 软件、TIM1_CH4、其他定时器事件等 |
| `TriggerEdge`     | 上升沿、下降沿或双边沿           |
| `Mode`            | 单次、连续、循环 DMA             |
| `BufferMode`      | 单值、线性缓冲、循环缓冲         |
| `ResultBuffer`    | 结果缓冲区地址                   |
| `Notification`    | 组完成通知函数                   |
| `Timeout`         | 可选转换超时阈值                 |

建议初始组定义：

| 组                            | 用途                 | 类型   | 触发方式       | 数据模式 |
| ----------------------------- | -------------------- | ------ | -------------- | -------- |
| `ADC_GROUP_REGULAR_MONITOR` | 母线电压、功率级温度 | 规则组 | 软件或周期任务 | DMA 循环 |
| `ADC_GROUP_FAST_CURRENT`    | PMSM 相电流          | 注入组 | TIM1_CH4       | 中断完成 |

## 6. 触发方式设计

### 6.1 软件触发

适用于：

- 诊断读取；
- 低速温度采样；
- 上电自检；
- 不要求固定采样相位的辅助通道。

推荐接口：

```c
Adc_Result Adc_StartSoftwareConversion(Adc_GroupType group);
```

### 6.2 定时器硬件触发

适用于：

- FOC 电流采样；
- 与 PWM 中心点同步的采样；
- 需要固定采样周期和低抖动的控制环路。

推荐接口：

```c
Adc_Result Adc_EnableHardwareTrigger(Adc_GroupType group);
Adc_Result Adc_DisableHardwareTrigger(Adc_GroupType group);
```

触发源不应散落在业务代码中，应由组配置绑定：

```text
ADC_GROUP_FAST_CURRENT -> TIM1_CH4_COMPARE
ADC_GROUP_REGULAR_MONITOR -> SOFTWARE
```

后续切换到 TIM8、TIM2 或其他触发源时，只修改组配置和底层映射。

### 6.3 连续转换和 DMA

连续 DMA 适用于慢速监控组或周期采样组，不建议默认用于 FOC 注入组。注入组应优先采用“定时器触发 + 转换完成中断 + 结果快照”的模式，保证控制环读取的是同一触发时刻的一组数据。

## 7. 推荐对外接口

### 7.1 类型定义

```c
typedef uint16_t Adc_ValueType;
typedef uint16_t Adc_ChannelType;
typedef uint16_t Adc_GroupType;

typedef enum
{
  ADC_STATUS_UNINIT = 0U,
  ADC_STATUS_IDLE,
  ADC_STATUS_BUSY,
  ADC_STATUS_COMPLETED,
  ADC_STATUS_ERROR
} Adc_StatusType;

typedef enum
{
  ADC_RESULT_OK = 0U,
  ADC_RESULT_NOT_INITIALIZED,
  ADC_RESULT_INVALID_CHANNEL,
  ADC_RESULT_INVALID_GROUP,
  ADC_RESULT_BUSY,
  ADC_RESULT_NOT_READY,
  ADC_RESULT_BUFFER_ERROR,
  ADC_RESULT_HAL_ERROR,
  ADC_RESULT_TIMEOUT
} Adc_Result;
```

### 7.2 基础生命周期接口

```c
void Adc_Init(void);
void Adc_DeInit(void);
Adc_StatusType Adc_GetStatus(void);
```

### 7.3 组控制接口

```c
Adc_Result Adc_StartGroupConversion(Adc_GroupType group);
Adc_Result Adc_StopGroupConversion(Adc_GroupType group);
Adc_Result Adc_GetGroupStatus(Adc_GroupType group,
                              Adc_StatusType *status);
```

### 7.4 结果读取接口

```c
Adc_Result Adc_SetupResultBuffer(Adc_GroupType group,
                                 Adc_ValueType *buffer,
                                 uint16_t length);
Adc_Result Adc_ReadGroup(Adc_GroupType group,
                         Adc_ValueType *buffer,
                         uint16_t length);
Adc_Result Adc_ReadChannel(Adc_ChannelType channel,
                           Adc_ValueType *value);
```

`Adc_ReadGroup()` 应复制一组完整结果，不应返回正在被 DMA 改写的中间状态。对于循环 DMA，建议采用双缓冲、半传输/全传输通知或临界区快照。

### 7.5 通知接口

```c
typedef void (*Adc_GroupNotificationType)(Adc_GroupType group);

Adc_Result Adc_EnableGroupNotification(Adc_GroupType group);
Adc_Result Adc_DisableGroupNotification(Adc_GroupType group);
void Adc_ConversionCompleteCallback(Adc_GroupType group);
```

中断服务程序只做最小处理：确认 HAL 事件、更新组状态、复制或标记结果、调用已注册通知。不得在中断中执行复杂换算或长时间阻塞。

## 8. 数据一致性与实时性要求

### 8.1 FOC 快速采样组

- 必须使用固定硬件触发；
- 组内通道顺序固定；
- 转换完成后一次性发布结果；
- 结果必须带有有效标志或序号；
- 禁止上层读取未完成转换的数据；
- 需要记录丢样、过载和 ADC 错误。

### 8.2 DMA 规则组

- 结果缓冲区长度必须与转换数量一致；
- 缓冲区应静态分配；
- DMA 循环模式下应定义半传输和全传输语义；
- 读取缓冲区时应避免与 DMA 同时访问；
- 不允许通过裸指针暴露 HAL DMA 内部缓冲区。

### 8.3 原始值与物理值分离

Board/Adc 只输出 ADC 原始计数值。例如：

```text
ADC raw value: 0 ... 4095
```

以下内容应位于上层：

```text
母线电压 = 原始值 × 分压比例 × 参考电压 / ADC 满量程
相电流 = 原始值经零点偏置和增益换算
温度 = 原始值经 NTC 查表或公式换算
```

## 9. 错误与诊断设计

建议至少定义以下错误位：

```text
ADC_ERROR_NOT_INITIALIZED
ADC_ERROR_INVALID_CONFIG
ADC_ERROR_INVALID_GROUP
ADC_ERROR_INVALID_BUFFER
ADC_ERROR_HAL
ADC_ERROR_DMA
ADC_ERROR_OVERRUN
ADC_ERROR_TIMEOUT
ADC_ERROR_TRIGGER_LOST
ADC_ERROR_RESULT_NOT_READY
```

建议提供：

```c
uint32_t Adc_GetErrorStatus(void);
void Adc_ClearErrorStatus(uint32_t mask);
uint32_t Adc_GetGroupError(Adc_GroupType group);
```

故障处理原则：

- ADC 初始化失败不得进入正常采样状态；
- 触发丢失应可被上层监测；
- DMA 错误和 ADC overrun 应记录并锁存，直到明确清除；
- 快速电流采样异常应通知安全管理或 FOC 状态机；
- ADC 驱动不直接执行功率级关闭，但必须提供可靠故障状态。

## 10. 初始化与运行时流程

### 10.1 初始化流程

```text
Adc_Init()
    |
    +-- 校验静态配置
    +-- 清零组状态和错误状态
    +-- 建立逻辑通道到硬件通道映射
    +-- 配置结果缓冲区
    +-- 配置组通知
    +-- 保持所有转换组停止
    v
ADC_IDLE
```

### 10.2 快速注入组流程

```text
PowerStage/PWM 初始化
    |
    +-- TIM1_CH4 已配置为 ADC 触发源
    +-- Adc_StartGroupConversion(ADC_GROUP_FAST_CURRENT)
    +-- Adc_EnableHardwareTrigger(ADC_GROUP_FAST_CURRENT)
    v
TIM1_CH4 事件
    v
ADC2 注入组转换
    v
ADC 中断
    v
Adc_ConversionCompleteCallback()
    v
发布 U/V/W 原始采样结果
```

### 10.3 规则 DMA 组流程

```text
Adc_StartGroupConversion(ADC_GROUP_REGULAR_MONITOR)
    v
ADC1 规则组
    v
DMA 循环写入结果缓冲区
    v
半传输/全传输通知
    v
Adc_ReadGroup()
```

## 11. 扩展规则

### 新增 ADC 通道

1. 增加逻辑 `ChannelId`；
2. 增加通道配置项；
3. 指定所属 `GroupId` 和 Rank；
4. 调整底层 CubeMX 通道配置；
5. 调整结果缓冲区长度；
6. 增加对应测试和诊断项；
7. 不修改已有 `Adc_ReadGroup()` 和组控制接口。

### 新增触发方式

1. 增加 `Adc_TriggerSource` 枚举；
2. 增加 HAL 触发源到逻辑触发源的映射；
3. 在组配置中选择触发源；
4. 实现触发使能、禁止和状态查询；
5. 增加触发丢失和边沿配置测试。

### 新增 ADC 实例

1. 增加 ADC 实例配置；
2. 建立实例到 HAL 句柄的映射；
3. 独立配置 DMA 和中断；
4. 不让上层直接依赖 `hadc3` 等句柄名称。

## 12. 测试要求

### 单元和接口测试

- 初始化前调用接口返回 `ADC_RESULT_NOT_INITIALIZED`；
- 无效 ChannelId 和 GroupId 被拒绝；
- 结果缓冲区为空或长度不足时返回错误；
- 组启动、停止和重复启动状态正确；
- 通知使能和禁止状态正确；
- 软件触发组可以完成一次转换；
- 硬件触发组能够响应 TIM1_CH4；
- ADC DMA 结果持续更新；
- ADC overrun、DMA 错误和触发丢失可记录。

### PMSM 板级验证

- ADC2 注入组在 TIM1_CH4 事件后进入完成回调；
- 注入组 3 个结果的顺序与配置 Rank 一致；
- ADC1 两路规则组 DMA 缓冲区持续更新；
- 停止 PWM 后，ADC 注入触发停止；
- PowerStage Break 后，ADC 快速采样组进入受控状态；
- 无高压条件下完成采样时序和中断验证；
- 使用示波器或逻辑分析仪确认 TIM1_CH4 与采样完成事件关系。

## 13. 分阶段实施计划

| 版本 | 实施内容                               | 出口条件             |
| ---- | -------------------------------------- | -------------------- |
| v0.1 | 建立文档、类型、逻辑通道和组模型       | 配置边界冻结         |
| v0.2 | 实现`Adc_Init`、组状态和基础结果读取 | 软件触发规则组可用   |
| v0.3 | 实现 ADC1 DMA 规则组                   | DMA 结果连续稳定     |
| v0.4 | 实现 ADC2 注入组和 TIM1_CH4 触发       | FOC 快速采样回调稳定 |
| v0.5 | 实现通知、错误、超时和触发状态         | 故障可诊断           |
| v0.6 | 接入 Driver_Test 和板级验证            | 测试记录完成         |
| v1.0 | 接入电流、电压、温度信号服务           | ADC 模块接口冻结     |

## 14. 设计结论

板级 `Adc` 模块应采用“逻辑通道 + ADC 组 + 静态配置 + 统一结果接口”的设计，不应围绕当前 3 个注入通道或当前 ADC1/ADC2 配置写死业务代码。

推荐优先实现以下最小闭环：

```text
Adc_Init()
Adc_SetupResultBuffer()
Adc_StartGroupConversion()
Adc_StopGroupConversion()
Adc_ReadGroup()
Adc_GetGroupStatus()
Adc_EnableGroupNotification()
Adc_ConversionCompleteCallback()
```

其中：

- ADC1 规则组负责电压、温度等监控采样；
- ADC2 注入组负责与 TIM1_CH4 同步的快速电流采样；
- HAL 句柄、DMA 句柄和 MCU 通道号只存在于 Board/Adc 实现层；
- 上层只依赖逻辑通道、逻辑组和原始采样结果；
- 所有新增通道和触发源都通过配置扩展，不修改既有公共接口。
