# PMSM D1 MCU 基础驱动实现方案

**版本**：v0.4
**适用平台**：STM32F407IGT6 + STM32CubeMX + STM32 HAL + CMake
**适用硬件**：ATK-DMF407 MCU 控制板 + ATK-PD6010B 功率驱动板
**关联阶段**：底层驱动实现顺序表 D1  语言：C
**文档性质**：D1 未完成部分的设计与代码实施方案

---

## 1. 方案目标

本方案用于完成 D1“MCU 基础驱动”中尚未完成的部分，并与以下两份架构基线保持一致：

- [VESC BLDC 源码架构地图](VESC_BLDC_Source_Architecture_Map_v0.1.md)
- [VESC BLDC 多硬件管理与 PMSM 硬件资源映射](VESC_BLDC_Hardware_Management_and_PMSM_Resource_Map_v0.1.md)

当前 D1 中需要补齐的主要内容：

```text
IWDG 服务/喂狗机制
复位原因读取
NVIC 中断实际响应验证
DMA 实际传输验证
GPIO 和系统时基实机验证
```

本方案不实现 PWM、ADC、FOC、通信协议和 VESC Tool 应用逻辑。D1 只负责建立稳定的 MCU 基础服务，为后续 D2～D8 提供统一的平台能力。

---

## 2. 与 VESC BLDC 架构的对应关系

VESC BLDC 使用 ChibiOS 提供 MCU 启动、HAL、RTOS、定时器、中断、看门狗和外设基础能力；板级实现通过 `board.c`、`hwconf/hw.h` 和相关硬件文件向公共模块提供统一入口。

PMSM 工程采用 STM32CubeMX 生成的 HAL/CMSIS，使用以下等价分层：

```text
VESC BLDC / ChibiOS                 PMSM / CubeMX
----------------------------------------------------------
board.c / __early_init()      ->    SystemClock_Config()
HAL 初始化                    ->    HAL_Init() + MX_*_Init()
ChibiOS GPIO/HAL              ->    STM32 HAL 外设初始化
ChibiOS 中断入口              ->    stm32f4xx_it.c
ChibiOS 看门狗服务            ->    PMSM Watchdog Service
ChibiOS 复位/启动信息         ->    PMSM Reset Cause Service
hw.h 统一硬件入口             ->    PMSM Board/HAL 统一接口
公共模块依赖硬件抽象          ->    D2～D8 只依赖 PMSM 基础服务
```

D1 的核心原则：

1. 不让 D2～D8 直接依赖散落在 `main.c` 中的 HAL 调用；
2. 保留 CubeMX 生成代码的边界，用户代码放入独立模块或 `USER CODE` 区域；
3. 不在 D1 中引入 ChibiOS API 兼容层；
4. 不让 D1 依赖 FOC、VESC 命令或通信协议；
5. 对功率级相关资源保持安全默认状态；
6. 基础服务的错误状态必须可被上层读取，但不直接实现上层故障策略。

---

## v0.2 实施记录

本版本已完成：

```text
[完成] reset.h/reset.c：读取 RCC 复位标志、解析复位原因、提供查询接口
[完成] main.c：HAL_Init() 后读取复位原因并清除硬件标志
[完成] time.h/time.c：提供毫秒 Tick、经过时间和周期到期判断接口
[完成] CMake：将 reset.c/time.c 加入应用源文件
[完成] 主循环：调用 Tick 接口，确认基础服务可被工程使用
```

当前未完成：

```text
[待验证] IWDG 实机复位和复位原因识别
[待验证] SysTick Tick 递增和溢出行为
[待验证] 复位原因覆盖上电、外部、软件和看门狗场景
[待完成] IWDG 基于心跳的完整服务框架
```

本版本只实现复位原因和 Tick 基础服务；IWDG 的完整心跳喂狗服务仍按 v0.3 计划实施。

---

## v0.3 实施记录

本版本已完成：

```text
[完成] watchdog.h/watchdog.c：IWDG 服务和基础心跳位图
[完成] 心跳注册、心跳上报和健康条件刷新 IWDG
[完成] 主循环注册 MAIN_LOOP/TIME_BASE 服务
[完成] 主循环报告心跳并调用 Watchdog_MainFunction()
[完成] CMake 加入 watchdog.c
```

当前实现边界：

```text
[已启用] MAIN_LOOP
[已启用] TIME_BASE
[未启用] DMA_SERVICE，待 DMA 驱动完成后接入
[待验证] IWDG 实际复位行为
[待验证] 心跳缺失时停止刷新并触发复位
```

说明：v0.3 仅实现 D1 基础服务心跳框架，不接入 ADC、PWM、FOC、通信和故障服务心跳。IWDG 初始化仍由 CubeMX 生成的 `MX_IWDG_Init()` 完成，Watchdog 模块只负责服务状态和受控刷新。

---

## v0.4 实施记录

本版本新增 D1 基础验证框架：

```text
[完成] verify.h/verify.c 独立验证模块
[完成] ADC1 规则组 DMA 启动和回调计数
[完成] ADC2 注入组启动和注入完成回调计数
[完成] ADC/DMA 错误计数
[完成] LED 周期翻转作为主循环/GPIO运行指示
[完成] CMake 加入 verify.c
[完成] 主循环接入 Verify_MainFunction()
```

当前验证模块职责：

```text
ADC1 DMA：验证规则组传输、半传输和完成回调
ADC2 注入组：验证 TIM1_CH4 触发及注入完成中断
DMA：统计 DMA 错误回调
GPIO：LED 每 500 ms 翻转，验证 GPIO 输出和主循环运行
NVIC：通过 ADC/DMA 中断回调统计验证中断入口
```

当前限制：

```text
[待实机] 下载运行并确认 LED 翻转
[待实机] 读取 VerifyStatus，确认 ADC/DMA/NVIC 计数递增
[待实机] 使用示波器/逻辑分析仪确认 ADC 触发频率
[待实机] 验证 GPIO 默认安全电平及 Gate 保持关闭
[待实机] 验证 DMA 错误注入和异常路径
```

说明：验证模块不启动 TIM1 PWM，因此 ADC2 的 TIM1_CH4 注入触发计数只有在 TIM1_CH4 已启动并产生比较事件后才会递增。该模块不执行 FOC、不使能 Gate，也不修改功率级输出。

---

## 3. 当前工程基线

当前工程已经具备以下基础配置：

| 项目                 | 当前状态                    | 主要位置                    |
| -------------------- | --------------------------- | --------------------------- |
| STM32 HAL 初始化     | 已完成基础配置              | `Core/Src/main.c`         |
| 系统时钟             | 已配置 168 MHz PLL          | `Core/Src/main.c`         |
| GPIO 初始化          | 已生成                      | `Core/Src/gpio.c`         |
| DMA 初始化           | 已生成                      | `Core/Src/dma.c`          |
| SysTick/HAL 时间基准 | 已由 HAL 配置               | `Core/Src/stm32f4xx_it.c` |
| IWDG 初始化          | 已生成                      | `Core/Src/iwdg.c`         |
| ADC、TIM1 中断入口   | 已生成                      | `Core/Src/stm32f4xx_it.c` |
| IWDG 服务            | v0.2 已完成基础接口和调用   | 待实机验证                  |
| 复位原因读取         | v0.2 已完成读取、解析和清除 | 待实机验证                  |
| 实机基础验证         | 待执行                      | 板级测试                    |

当前启动顺序位于 [main.c](../software/PMSM_FOC/Core/Src/main.c)：

```text
HAL_Init()
    -> SystemClock_Config()
    -> MX_GPIO_Init()
    -> MX_DMA_Init()
    -> MX_TIM1_Init()
    -> MX_IWDG_Init()
    -> MX_ADC1_Init()
    -> MX_ADC2_Init()
```

D1 完成后，不改变上述 CubeMX 外设初始化职责，只增加基础服务初始化、主循环服务和复位信息采集。

---

## 4. D1 目标软件分层

建议 D1 形成以下软件边界：

```text
+--------------------------------------------------+
| D2～D8 功率、采样、故障、通信、存储驱动         |
+------------------------+-------------------------+
                         |
              PMSM MCU Basic Services
                         |
+------------------------+-------------------------+
| Reset Cause | Watchdog | Time Base | DMA Status  |
+------------------------+-------------------------+
                         |
+------------------------v-------------------------+
| STM32 HAL / CMSIS / CubeMX Generated Code        |
+--------------------------------------------------+
```

建议新增或预留的模块边界：

| 模块         | 主要职责                   | 禁止职责                |
| ------------ | -------------------------- | ----------------------- |
| `reset`    | 读取并保存 MCU 复位原因    | 处理电机故障策略        |
| `watchdog` | 检查系统心跳并刷新 IWDG    | 在任意 ISR 中无条件喂狗 |
| `time`     | 提供毫秒时基和周期判断     | 承担 FOC 高频计时       |
| `dma`      | 提供 DMA 状态和错误信息    | 直接解释 ADC 物理量     |
| `mcu`      | D1 基础服务统一初始化      | 初始化上层控制算法      |
| `main.c`   | 按顺序调用初始化和周期服务 | 堆叠具体硬件业务逻辑    |

---

## 5. IWDG 服务实施方案

### 5.1 设计目标

当前 [iwdg.c](../software/PMSM_FOC/Core/Src/iwdg.c) 已完成 IWDG 初始化，但 [main.c](../software/PMSM_FOC/Core/Src/main.c) 尚未建立喂狗服务。

IWDG 服务必须满足：

- 只有系统主循环在健康条件下才能刷新看门狗；
- 不能由单独一个中断无条件刷新看门狗；
- 后续 D2～D8 可以通过心跳报告自身运行状态；
- FOC、ADC DMA、通信和故障服务异常时，主循环最终停止喂狗；
- 复位后可以通过复位原因服务识别 IWDG 复位。

### 5.2 推荐状态模型

```text
RESET
  -> WATCHDOG_INIT
  -> WAIT_FOR_HEARTBEATS
  -> SERVICE_ALLOWED
  -> SERVICE_SUSPENDED
  -> WATCHDOG_RESET
```

D1 阶段只实现基础状态和接口，不要求 D2～D8 立即提供全部心跳。未接入的服务可以由 D1 配置为“未启用”，不能默认为“永远健康”。

### 5.3 推荐接口边界

接口只表达设计意图，具体声明和实现放在后续代码实施阶段：

```c
void Watchdog_Init(void);
void Watchdog_RegisterHeartbeat(uint32_t service_id);
void Watchdog_Alive(uint32_t service_id);
void Watchdog_MainFunction(void);
void Watchdog_Suspend(void);
```

实施要求：

1. `Watchdog_MainFunction()` 只在主循环中调用；
2. 只有所有已启用服务满足心跳条件时，才调用 `HAL_IWDG_Refresh()`；
3. 初始化阶段必须明确是否允许喂狗；
4. 调试模式下是否暂停看门狗必须通过明确编译配置控制，不能隐式处理；
5. 不允许在 ADC、TIM1 Break 或 DMA ISR 中直接刷新 IWDG；
6. 所有局部变量按项目约定显式初始化；
7. API 注释使用 Doxygen 风格。

### 5.4 D1 阶段心跳范围

D1 只注册基础服务：

```text
MAIN_LOOP
MCU_TIME_BASE
DMA_SERVICE（若启用）
```

D2～D8 完成后再扩展：

```text
PWM_SERVICE
ADC_INJECTED_SERVICE
ADC_REGULAR_SERVICE
FAULT_SERVICE
POSITION_SERVICE
COMM_SERVICE
FLASH_SERVICE
```

### 5.5 验证顺序

```text
正常运行并持续喂狗
    -> 暂停主循环喂狗
    -> IWDG 复位
    -> 上电读取复位原因
    -> 确认识别为 IWDG 复位
    -> 清除/保存原因
```

测试时必须保持功率级 Gate 关闭，不得将看门狗测试与电机运行测试绑定。

---

## 6. 复位原因读取实施方案

### 6.1 设计目标

复位原因读取应尽早执行，位置应在 `HAL_Init()` 之后、复杂外设初始化之前，避免后续初始化覆盖或影响诊断信息。

建议主线：

```text
复位入口
    -> HAL_Init()
    -> Reset_ReadCause()
    -> Reset_StoreCause()
    -> Reset_ClearHardwareFlags()
    -> SystemClock_Config()
    -> 其他外设初始化
```

### 6.2 复位原因范围

至少识别 STM32F407 的以下复位来源：

| 复位原因   | 用途                         |
| ---------- | ---------------------------- |
| IWDG 复位  | 判断系统是否因看门狗超时复位 |
| WWDG 复位  | 预留窗口看门狗诊断           |
| 软件复位   | 区分软件主动复位             |
| POR/PDR    | 区分上电或掉电复位           |
| BOR        | 判断欠压复位                 |
| 外部 NRST  | 判断外部复位                 |
| 低功耗复位 | 预留低功耗场景               |

实际枚举以 STM32F4 HAL/RCC 标志宏为准，不在文档阶段硬编码具体寄存器操作。

### 6.3 推荐数据模型

建议建立只读诊断数据：

```text
原始 RCC 标志
解析后的复位原因
复位发生计数（可选）
上一次复位原因有效标志
```

D1 阶段不将复位原因写入 Flash，避免基础启动阶段引入 Flash 擦写和实时性问题。Flash 持久化属于 D8。

### 6.4 推荐接口边界

```c
void Reset_Init(void);
uint32_t Reset_GetRawFlags(void);
ResetCause Reset_GetCause(void);
void Reset_ClearFlags(void);
```

要求：

- 读取动作只执行一次或具备明确幂等性；
- 清除动作必须在读取和保存解析结果之后执行；
- 不能将复位原因与 VESC Fault Code 混为一类；
- 后续 D5 可以将 IWDG 复位映射为系统诊断事件，但不得覆盖原始复位原因。

---

## 7. NVIC 中断实际验证方案

当前中断入口已由 CubeMX 生成，位于 [stm32f4xx_it.c](../software/PMSM_FOC/Core/Src/stm32f4xx_it.c)。D1 的代码目标不是新增业务 ISR，而是确认基础中断路径可靠。

### 7.1 验证范围

```text
ADC_IRQHandler
TIM1_BRK_TIM9_IRQHandler
TIM1_UP_TIM10_IRQHandler
DMA2_Stream0_IRQHandler
SysTick_Handler
```

### 7.2 验证原则

- ISR 只负责调用对应 HAL Handler 和记录最小事件；
- 不在 D1 验证中执行 FOC；
- 不在 ISR 中进行 Flash 操作、阻塞等待或复杂日志输出；
- 使用 volatile 事件计数器或测试 GPIO 观察中断是否进入；
- 测试变量应放在独立测试模块，不污染生产接口。

### 7.3 建议验证项目

| 中断         | 验证方法                          | 通过条件                 |
| ------------ | --------------------------------- | ------------------------ |
| SysTick      | 读取系统 Tick                     | Tick 持续递增            |
| DMA2 Stream0 | 启动 ADC1 DMA 后观察传输回调/计数 | DMA 传输完成事件持续产生 |
| ADC          | 启动 ADC 注入组后观察注入完成事件 | 事件频率与 TIM1_CC4 一致 |
| TIM1 Break   | 外部触发 BKIN                     | PWM 被硬件关闭并进入中断 |
| TIM1 Update  | 仅在明确启用更新中断后验证        | 频率与 RCR/计数模式一致  |

D1 只验证中断基础通路；ADC 采样内容、PWM/ADC 时序和 Break 故障策略属于 D2/D3/D5 的正式验证。

---

## 8. DMA 实际传输验证方案

当前 ADC1 DMA 配置用于规则组采样。D1 只验证 DMA 传输链路，不实现母线电压和 MOS 温度换算。

### 8.1 验证链路

```text
ADC1 规则组
    -> DMA2 Stream0
    -> ADC 缓冲区
    -> DMA 完成回调
    -> 传输计数/错误状态
```

### 8.2 代码实施边界

建议建立 DMA 服务状态：

```text
未启动
运行中
传输完成
半传输（如启用）
传输错误
停止
```

状态服务不应解释 ADC 数据含义。ADC 数据的物理量解析属于 D4。

### 8.3 验证项目

| 项目       | 验证内容                                   |
| ---------- | ------------------------------------------ |
| 缓冲区地址 | DMA 目标地址有效且生命周期覆盖整个采样过程 |
| 数据长度   | 与 ADC 规则组 Rank 数一致                  |
| 数据更新   | 缓冲区内容随输入变化                       |
| 传输完成   | 回调或事件计数持续变化                     |
| DMA 错误   | 人为制造/模拟错误后状态可读取              |
| 停止恢复   | 停止后重新启动不产生非法状态               |

D1 期间不允许在 DMA 回调中直接调用 FOC、Flash 或通信协议处理。

---

## 9. GPIO 与系统时基实机验证方案

### 9.1 GPIO 验证

D1 重点确认：

- 复位期间 Gate/SD 保持关闭；
- PWM 引脚不会输出非预期窄脉冲；
- BKIN 输入状态稳定；
- ADC 引脚处于模拟输入模式；
- SWD 引脚保持调试功能；
- 状态 LED 可用于基础运行指示。

D1 不修改 D2 的 PWM 启动策略。任何功率级使能验证必须经过 D2 安全启动方案。

### 9.2 系统时基验证

SysTick 由 HAL 管理，D1 只提供统一的毫秒时间读取接口：

```c
uint32_t Time_GetTickMs(void);
```

验证项目：

```text
Tick 递增
计时无明显跳变
系统 Tick 溢出处理规则明确
不能在 FOC ISR 中使用阻塞延时
不能用 HAL_Delay() 实现控制周期
```

### 9.3 LED 指示建议

LED 只用于基础诊断，不承担安全控制：

```text
常亮：基础初始化完成
慢闪：主循环运行
快闪：基础服务异常
故障状态：由 D5 接管
```

LED 控制必须与 Gate/PWM 控制解耦。

---

## 10. `main.c` 初始化和周期调用方案

D1 完成后的 `main.c` 应保持初始化顺序清晰，建议结构如下：

```text
HAL_Init()
    -> Reset_Init()
    -> SystemClock_Config()
    -> MX_GPIO_Init()
    -> MX_DMA_Init()
    -> MX_IWDG_Init()
    -> Mcu_Init()
    -> 其他外设 MX_*_Init()
    -> Watchdog_Start()
    -> 进入主循环

主循环：
    -> Time_MainFunction()
    -> Dma_MainFunction()
    -> Watchdog_MainFunction()
    -> 基础状态指示
```

注意：

1. D1 阶段不启动 PWM、ADC 注入组和 Gate；
2. 后续 D2/D3 完成后再将相关启动动作加入统一的启动状态机；
3. `main.c` 只负责初始化编排，不承载具体驱动细节；
4. CubeMX 重新生成代码时，用户代码必须位于保留区域或独立源文件；
5. 所有初始化失败都必须进入统一错误处理路径。

---

## 11. 错误处理和安全要求

### 11.1 D1 基础错误分类

```text
MCU_OK
MCU_INIT_ERROR
MCU_CLOCK_ERROR
MCU_DMA_ERROR
MCU_TIME_ERROR
MCU_RESET_DIAG_ERROR
MCU_WATCHDOG_ERROR
```

具体枚举名称在代码实施阶段确定，不能与 VESC 电机故障码混用。

### 11.2 错误处理原则

- 时钟初始化失败：进入不可恢复安全错误；
- DMA 基础配置失败：禁止启动依赖 DMA 的采样服务；
- 复位原因读取失败：保留原始寄存器信息并报告诊断状态；
- IWDG 初始化失败：禁止进入后续功率级使能流程；
- 主循环失去调度：允许 IWDG 执行最终复位；
- D1 错误不能直接执行功率级恢复动作。

### 11.3 功率级安全默认值

与 VESC `hw.h` 中 `ENABLE_GATE()`/`DISABLE_GATE()` 的硬件动作宏对应，PMSM D1 必须保证：

```text
MCU 复位 -> Gate 关闭
HAL 初始化期间 -> Gate 关闭
D1 初始化失败 -> Gate 关闭
看门狗复位前 -> 不主动开启 Gate
系统复位后 -> 重新确认 Gate 关闭
```

---

## 12. 代码实施顺序

| 顺序 | 实施内容                         | 依赖         | 结果                     |
| ---: | -------------------------------- | ------------ | ------------------------ |
|    1 | 建立 D1 基础服务目录和头文件边界 | D0           | 模块边界明确             |
|    2 | 实现复位原因读取                 | HAL/RCC      | 上电可获得复位诊断       |
|    3 | 实现系统 Tick 访问和基础时间服务 | SysTick      | 统一时间接口             |
|    4 | 实现 DMA 状态和错误记录          | DMA          | 可观察 DMA 生命周期      |
|    5 | 实现 IWDG 服务框架               | IWDG、主循环 | 具备受控喂狗能力         |
|    6 | 接入基础心跳机制                 | 时间服务     | 可判断基础服务健康状态   |
|    7 | 完成 GPIO 安全状态验证代码       | GPIO         | 确认复位和初始化默认状态 |
|    8 | 完成 NVIC/DMA/SysTick 实机测试   | 中断和 DMA   | 基础中断链路可验证       |
|    9 | 形成 D1 测试记录                 | 全部 D1 内容 | 具备阶段审查依据         |
|   10 | 更新 D1 阶段状态                 | 测试记录     | D1 完成或保留未完成项    |

---

## 13. D1 阶段验收标准

D1 只有在以下条件全部满足后，才可标记为“完成”：

```text
[ ] 工程能够稳定启动
[ ] 时钟频率已通过实测或可靠工具确认
[ ] GPIO 复位默认状态符合安全要求
[ ] Gate/SD 在初始化和错误路径中保持关闭
[ ] SysTick 持续递增
[ ] 基础时间接口可用
[ ] NVIC 中断入口实际响应
[ ] DMA 实际传输成功
[ ] DMA 错误状态可识别
[ ] IWDG 正常运行
[ ] IWDG 喂狗由健康条件控制
[ ] 停止喂狗后能够复位
[ ] 能识别 IWDG 复位原因
[ ] 能识别上电、外部和软件复位原因
[ ] 复位标志读取后按规定清除
[ ] 基础测试不启动 PWM 和 Gate
[ ] 测试记录已归档
```

任何一项未完成，D1 应标记为“部分完成”，不能进入 D2～D8 的正式功率级联调阶段。

---

## 14. 与后续阶段的接口交付物

D1 完成后向后续阶段交付：

| 交付物                   | 使用阶段   |
| ------------------------ | ---------- |
| 系统初始化顺序           | D2～D8     |
| 统一毫秒时间接口         | D3～D8     |
| DMA 状态接口             | D3、D4     |
| IWDG 心跳机制            | D5、D7、D9 |
| 复位原因数据             | D5、D8、D9 |
| GPIO 安全默认状态        | D2、D5     |
| 中断优先级和基础验证记录 | D2、D3、D5 |
| D1 测试报告和版本基线    | D9         |

与 VESC BLDC 的对应关系是：D1 先提供类似 `board.c`、ChibiOS HAL 和系统服务的基础能力；D2～D8 再分别实现 `hwconf` 中的 PWM、ADC、Gate、保护、编码器和通信资源，不把所有板级逻辑集中到一个 `hw.h` 文件中。

---

## 15. 版本与变更控制

建议本方案实施过程使用以下版本：

```text
v0.1：D1 设计方案
v0.2：完成复位原因和 Tick 基础服务
v0.3：完成 IWDG 服务和心跳框架
v0.4：完成 DMA/NVIC/GPIO 实机验证
v0.5：D1 阶段验收和冻结
```

D1 冻结后，以下内容不得在 D2 阶段无记录修改：

- 系统时钟和 APB 分频；
- NVIC 优先级分组；
- 基础时间基准；
- IWDG 超时时间；
- Gate 默认安全状态；
- DMA 中断基础配置；
- 复位原因读取和清除策略。
