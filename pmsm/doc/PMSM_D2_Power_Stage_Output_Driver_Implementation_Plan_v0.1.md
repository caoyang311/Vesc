# PMSM D2 功率级输出驱动完善落地方案

**版本**：v0.1
**适用平台**：STM32F407IGT6 + STM32CubeMX + STM32 HAL + CMake
**适用硬件**：ATK-DMF407 MCU 控制板 + ATK-PD6010B 功率驱动板
**关联阶段**：PMSM 底层软件驱动实现顺序表 D2
**前置阶段**：D1 MCU 基础驱动已完成基础项验证
**文档性质**：D2 功率级输出驱动完善、代码实施和板级验收方案

## v0.6 实施记录

本版本已完成：

```text
[完成] 增加 PowerStage_ScopeVerificationStart() 验证接口
[完成] 验证接口按 SAFE_OFF -> READY -> START -> RUNNING 顺序执行
[完成] 复用 v0.4 固定占空比 U=25%、V=50%、W=75%
[完成] 在 main 初始化完成后调用无高压示波器验证接口
[完成] 启动 TIM1 CH4 作为 ADC2 注入转换触发输出
[完成] 停止 TIM1 CH4，确保 PWM 停止时 ADC 触发同步停止
[完成] 启动失败时维持 Gate/SD 关闭和故障锁存
```

示波器验证要求：

```text
[ ] 上电后确认三相主 PWM 和互补 PWM 均存在
[ ] 确认 U/V/W 占空比分别约为 25%/50%/75%
[ ] 确认互补输出不重叠
[ ] 确认 Dead Time 波形
[ ] 确认 PM1_CTRL_SD 在 PWM 稳定后有效
[ ] 确认不接高压母线时无功率级异常
[ ] 验证完成后调用 PowerStage_Stop() 关闭输出
```

---

## v0.5 实施记录

本版本已完成：

```text
[完成] 实现 HAL_TIMEx_BreakCallback() 回调
[完成] 仅处理 TIM1 Break 事件，忽略其他定时器回调
[完成] Break 回调锁存 POWER_STAGE_FAULT_BREAK
[完成] Break 回调停止六路 PWM 并关闭 Gate/SD
[完成] PowerStage_ClearFault() 检查 TIM1 Break 标志
[完成] Break 条件仍有效时禁止清除故障
[完成] 清除前重新确保六路 PWM 停止且 Gate/SD 关闭
[完成] 清除成功后回到 SAFE_OFF，禁止自动重启
```

当前边界：

```text
[待验证] BKIN 释放后的实际硬件清除行为
[待验证] Break 中断、硬件输出关断和软件锁存的一致性
[已实现] PowerStage_Start() 在 PWM 成功启动后重新使能 Gate/SD
[待实施] 清除后的重新 Ready/Start 流程测试
```

---

## v0.4 实施记录

本版本已完成：

```text
[完成] PowerStage_Start() 设置固定三相测试比较值
[完成] PowerStage_Start() 启动 TIM1_CH1/CH2/CH3 主 PWM
[完成] PowerStage_Start() 启动 TIM1_CH1N/CH2N/CH3N 互补 PWM
[完成] PowerStage_SetDuty() 使用 __HAL_TIM_SET_COMPARE() 更新 CCR1/CCR2/CCR3
[完成] PWM 启动失败时停止全部 PWM、关闭 Gate/SD 并锁存故障
[完成] PWM 启动成功后进入 RUNNING 状态
```

当前边界：

```text
[保持关闭] Gate/SD 开启控制，待后续版本实施
[待实施] Break 清除条件与受控恢复
[待验证] 六路 PWM 波形、互补关系和固定占空比
```

---

## v0.3 实施记录

本版本已完成：

```text
[完成] PowerStage 初始化时关闭 TIM1 六路 PWM 输出
[完成] PowerStage 初始化时关闭 Gate/SD
[完成] PowerStage_Stop() 统一执行 PWM 停止和 Gate/SD 关闭
[完成] TIM1 Break 中断路径调用 PowerStage_BreakHandler()
[完成] Break 处理执行 PWM 停止、Gate/SD 关闭和故障锁存
[完成] HAL 停止异常进入 HAL 故障锁存状态
```

当前未完成：

```text
[待实施] PWM 启动和互补输出启动
[待实施] Gate/SD 开启接口
[待实施] 运行时占空比写入 CCR
[待实施] Break 清除条件与受控恢复
[待验证] 复位、初始化和停止波形
```

---

## 1. 方案目标

本方案用于将当前 D2 从“TIM1、互补 PWM、Dead Time、BKIN 和 Gate/SD 默认电平已配置”完善为“具备独立控制接口、明确安全时序、可处理 Break 故障并完成无高压板级验证”的功率级输出驱动。

D2 只负责功率级输出和安全关断，不实现以下内容：

```text
ADC 采样数据解析
FOC/Clarke/Park/SVPWM 算法
电流环、速度环和位置环
过流/过压/过温阈值策略
VESC 协议和通信应用
```

D2 的设计目标：

1. PWM 默认关闭，Gate/SD 默认关闭；
2. 所有 PWM 输出必须通过统一接口启动和停止；
3. 运行时占空比只能通过受约束接口更新；
4. Break 触发后硬件立即关断，软件记录并锁存故障；
5. 故障状态下禁止重新开启 PWM 和 Gate；
6. D2 代码不直接混入 FOC 或应用层；
7. 在不接高压母线的条件下完成六路 PWM、死区、Gate 和 Break 验证。

---

## 2. 当前工程基线

当前工程已经具备以下配置：

| 项目              | 当前状态                              | 位置                                 |
| ----------------- | ------------------------------------- | ------------------------------------ |
| TIM1 中心对齐计数 | 已配置                                | `Core/Src/tim.c`                   |
| TIM1 PWM 频率     | 已配置为 10 kHz 参数                  | `Core/Src/tim.c`、`PMSM_FOC.ioc` |
| 三相高侧 PWM      | PA8/PA9/PA10，TIM1_CH1/CH2/CH3        | `Core/Src/tim.c`                   |
| 三相低侧互补 PWM  | PB13/PB14/PB15，TIM1_CH1N/CH2N/CH3N   | `Core/Src/tim.c`                   |
| Break 输入        | PB12，TIM1_BKIN，低有效               | `Core/Src/tim.c`                   |
| Dead Time         | CubeMX/HAL 参数已配置为 84            | `Core/Src/tim.c`                   |
| Gate/SD           | PF10，`PM1_CTRL_SD`，默认输出低电平 | `Core/Src/gpio.c`                  |
| PWM 启停接口      | 未实现                                | 当前工程未调用 PWM Start API         |
| 运行时占空比接口  | 未实现                                | 当前仅在 TIM1 初始化中设置固定 Pulse |
| Break 软件处理    | 未实现                                | 仅有 HAL TIM 中断入口                |

当前 `main.c` 只调用 `MX_TIM1_Init()`，未调用 `HAL_TIM_PWM_Start()` 或 `HAL_TIMEx_PWMN_Start()`；因此 D2 不能仅依据 CubeMX 配置标记为完成。

---

## 3. D2 软件分层

```text
+---------------------------------------------------+
| D3/D4/D5/FOC                                      |
| 只使用 PowerStage 对外接口                       |
+-------------------------+-------------------------+
                          |
              PMSM Power Stage Service
                          |
+-------------------------+-------------------------+
| PWM 控制 | Gate/SD 控制 | Break 故障 | 状态查询   |
+-------------------------+-------------------------+
                          |
+---------------------------------------------------+
| STM32 HAL TIM1/GPIO/CMSIS                          |
+---------------------------------------------------+
```

建议新增独立模块：

```text
Drivers/PowerStage/power_stage.h
Drivers/PowerStage/power_stage.c
```

模块职责：

| 模块               | 允许职责                                                      | 禁止职责                              |
| ------------------ | ------------------------------------------------------------- | ------------------------------------- |
| `power_stage`    | 初始化、PWM 启停、Gate 控制、占空比更新、Break 锁存、状态查询 | FOC 算法、ADC 物理量换算、通信协议    |
| `tim.c`          | CubeMX 生成的 TIM1 基础配置和 MSP 配置                        | 承载运行时功率级状态机                |
| `gpio.c`         | CubeMX 生成的 GPIO 默认配置                                   | 承载故障策略                          |
| `main.c`         | 初始化编排和周期服务调用                                      | 直接操作 TIM1/Gate 引脚               |
| `stm32f4xx_it.c` | 转发 TIM1 中断                                                | 在 ISR 中执行复杂恢复、日志或阻塞操作 |

---

## 4. 功率级状态模型

```text
RESET
  -> SAFE_OFF
  -> READY
  -> STARTING
  -> RUNNING
  -> STOPPING
  -> SAFE_OFF

任意状态 -- Break/软件故障 --> FAULT_LATCHED
FAULT_LATCHED -- 显式复位且 Break 已释放 --> SAFE_OFF
```

状态定义：

| 状态              | PWM      | Gate/SD            | 允许动作                            |
| ----------------- | -------- | ------------------ | ----------------------------------- |
| `SAFE_OFF`      | 关闭     | 关闭               | 初始化、查询、故障复位条件检查      |
| `READY`         | 关闭     | 关闭               | 安全启动请求                        |
| `STARTING`      | 启动中   | 按硬件要求受控开启 | 只允许内部启动流程                  |
| `RUNNING`       | 运行     | 开启               | 占空比更新、受控停止                |
| `STOPPING`      | 关闭中   | 关闭               | 只允许内部停止流程                  |
| `FAULT_LATCHED` | 强制关闭 | 强制关闭           | 查询故障、清除故障，不允许 PWM 启动 |

### 4.1 推荐状态枚举

```c
typedef enum
{
    POWER_STAGE_STATE_SAFE_OFF = 0U,
    POWER_STAGE_STATE_READY,
    POWER_STAGE_STATE_STARTING,
    POWER_STAGE_STATE_RUNNING,
    POWER_STAGE_STATE_STOPPING,
    POWER_STAGE_STATE_FAULT_LATCHED
} PowerStage_State;
```

### 4.2 推荐故障位

```c
typedef uint32_t PowerStage_FaultMask;

#define POWER_STAGE_FAULT_NONE       (0U)
#define POWER_STAGE_FAULT_BREAK      (1UL << 0U)
#define POWER_STAGE_FAULT_STARTUP    (1UL << 1U)
#define POWER_STAGE_FAULT_PARAMETER  (1UL << 2U)
#define POWER_STAGE_FAULT_HAL        (1UL << 3U)
```

D2 只定义和记录功率级故障。过流、过压、过温等保护阈值和诊断归属 D5；D5 可将外部保护结果映射到功率级安全停止接口。

---

## 5. 对外接口设计

接口只暴露功率级行为，不暴露 `TIM_HandleTypeDef` 和 GPIO 细节。

```c
void PowerStage_Init(void);
PowerStage_Result PowerStage_Ready(void);
PowerStage_Result PowerStage_Start(void);
PowerStage_Result PowerStage_Stop(void);
PowerStage_Result PowerStage_SetDuty(PowerStage_Duty duty);
PowerStage_Result PowerStage_ClearFault(void);
PowerStage_State PowerStage_GetState(void);
PowerStage_FaultMask PowerStage_GetFaults(void);
void PowerStage_BreakHandler(void);
```

建议数据类型：

```c
typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} PowerStage_Duty;

typedef enum
{
    POWER_STAGE_OK = 0U,
    POWER_STAGE_ERROR,
    POWER_STAGE_INVALID_STATE,
    POWER_STAGE_INVALID_DUTY,
    POWER_STAGE_FAULT_ACTIVE
} PowerStage_Result;
```

接口规则：

1. `PowerStage_Init()` 只建立软件状态并强制进入安全关闭状态；
2. `PowerStage_Ready()` 只允许在无锁存故障、Break 已释放且 Gate 关闭时执行；
3. `PowerStage_Start()` 必须先确认状态为 `READY`，再按规定顺序开启 PWM 和 Gate；
4. `PowerStage_Stop()` 必须先关闭 PWM，再关闭 Gate/SD；
5. `PowerStage_SetDuty()` 只允许在 `RUNNING` 状态调用；
6. `PowerStage_ClearFault()` 不得绕过硬件 Break，只有 Break 释放且 PWM/Gate 已关闭时才能清除；
7. `PowerStage_BreakHandler()` 只执行最小化关断和故障锁存动作；
8. 所有局部变量显式初始化，结构体使用 `{0}` 初始化；
9. 所有 `if` 使用配对 `else`；
10. 所有公共接口使用 Doxygen 注释。

---

## 6. PWM 参数和占空比策略

### 6.1 计数范围

当前 TIM1 周期由：

```text
TIM1 定时器时钟 = 168 MHz
PWM 频率 = 10 kHz
中心对齐模式
ARR = 168 MHz / 10 kHz / 2 = 8400
```

实施时不在 `PowerStage` 中重复计算 ARR，而是通过 `htim1.Init.Period` 或统一配置宏获得周期值，避免配置漂移。

### 6.2 占空比约束

推荐使用 `0 ... ARR` 的定时器比较值作为 D2 内部接口单位。D2 不引入浮点数。

```text
0       <= duty.u <= ARR
0       <= duty.v <= ARR
0       <= duty.w <= ARR
```

D2 阶段不实现电压矢量调制，只验证固定占空比和三相独立更新。

### 6.3 更新原则

- 三相占空比必须在同一 PWM 更新边界生效；
- 优先使用 CCR 预装载，不在高频 ISR 中执行复杂计算；
- 禁止将任意值直接写入 CCR；
- 不允许在 `SAFE_OFF`、`READY` 或 `FAULT_LATCHED` 状态更新有效输出；
- 初始占空比使用安全中点值或项目定义的安全值，禁止使用当前固定测试值直接作为正式运行默认值。

### 6.4 待确认硬件参数

在正式实现前必须确认：

1. `PM1_CTRL_SD` 的有效电平：高电平有效和命名含义：Gate/SD使能；
2. Gate Driver 的开启延时和关闭延时要求：立即开启与关断；
3. BKIN 外部电路的有效电平：低电平、滤波：5ms和复位条件：高电平持续100ms；
4. Dead Time=84 对应的实际时间:1us；
5. 功率板是否要求先 Gate/SD 开启再 PWM。

未确认前，只允许使用示波器和低风险测试工装，不允许接入高压母线和电机负载。

---

## 7. 启动、停止和故障时序

### 7.1 安全启动

```text
确认 SAFE_OFF/READY
    -> 确认 Break 未激活
    -> 清零/装载安全占空比
    -> 清除 TIM1 旧 Break 状态（仅在允许条件下）
    -> 启动 CH1/CH2/CH3 PWM
    -> 启动 CH1N/CH2N/CH3N 互补输出
    -> 等待一个受控 PWM 边界
    -> 开启 Gate/SD
    -> 状态置为 RUNNING
```

启动失败必须执行：

```text
关闭全部 PWM
    -> 关闭 Gate/SD
    -> 锁存 STARTUP/HAL 故障
    -> 状态置为 FAULT_LATCHED
```

### 7.2 安全停止

```text
停止占空比更新
    -> 关闭全部 PWM 主输出
    -> 关闭全部 PWM 互补输出
    -> 关闭 Gate/SD
    -> 状态置为 SAFE_OFF
```

### 7.3 Break 故障

```text
BKIN 触发
    -> TIM1 硬件关闭输出
    -> TIM1 Break ISR 进入
    -> 记录 BREAK 故障
    -> 软件关闭 Gate/SD
    -> 状态置为 FAULT_LATCHED
    -> 禁止自动恢复和自动重启
```

Break ISR 不执行：

```text
Flash 操作
阻塞等待
复杂日志
PWM 重新启动
故障自动清除
```

---

## 8. Break 处理设计

### 8.1 硬件层

现有 TIM1 已配置：

```text
BreakState = ENABLE
BreakPolarity = LOW
OffStateRunMode = OSSR_ENABLE
OffStateIDLEMode = OSSI_ENABLE
AutomaticOutput = ENABLE
```

这些配置只能提供硬件关断基础，不能替代软件故障记录、Gate 关闭和故障锁存。

### 8.2 软件层

推荐在 `HAL_TIMEx_BreakCallback()` 或等效 HAL 回调中调用：

```c
void PowerStage_BreakHandler(void);
```

处理顺序：

1. 设置 `POWER_STAGE_FAULT_BREAK`；
2. 设置状态 `POWER_STAGE_STATE_FAULT_LATCHED`；
3. 关闭 `PM1_CTRL_SD`；
4. 不启动任何 PWM；
5. 保留故障位供 D5/D9 查询。

### 8.3 故障清除

只有同时满足以下条件，才允许清除：

```text
BKIN 输入已释放
TIM1 Break 标志已按芯片要求清除
PWM 主输出和互补输出均已关闭
Gate/SD 已关闭
调用方明确请求清除
```

D2 不实现自动重启。故障恢复由 D5 的安全状态机决定。

---

## 9. `main.c` 集成顺序

D2 实施后建议保持以下顺序：

```text
HAL_Init()
    -> Reset_Init()
    -> SystemClock_Config()
    -> MX_GPIO_Init()
    -> MX_DMA_Init()
    -> MX_TIM1_Init()
    -> MX_IWDG_Init()
    -> MX_ADC1_Init()
    -> MX_ADC2_Init()
    -> PowerStage_Init()
    -> PowerStage_Ready()
    -> 进入 SAFE_OFF/READY
```

D2 验证阶段：

```text
PowerStage_Start() 仅由专用验证流程调用
PowerStage_SetDuty() 仅使用固定测试占空比
PowerStage_Stop() 在测试结束或异常时调用
```

`main.c` 不直接调用：

```c
HAL_TIM_PWM_Start(...);
HAL_TIMEx_PWMN_Start(...);
HAL_GPIO_WritePin(PM1_CTRL_SD_GPIO_Port, ...);
__HAL_TIM_SET_COMPARE(...);
```

这些操作必须集中在 `power_stage.c` 中。

---

## 10. 实施分阶段计划

| 版本 | 实施内容                                        | 完成标志                                 |
| ---- | ----------------------------------------------- | ---------------------------------------- |
| v0.2 | 建立`power_stage.h/.c`、状态和结果类型        | 模块可编译，接口边界冻结                 |
| v0.3 | 实现 Gate/SD 安全控制和 PWM Stop                | 上电、初始化失败、停止路径均保持安全关闭 |
| v0.4 | 实现 PWM Start、互补输出 Start 和固定占空比更新 | 六路输出可以受控启动和停止               |
| v0.5 | 实现 Break 回调、故障锁存和清除条件             | Break 后 PWM/Gate 关闭且不可自动恢复     |
| v0.6 | 完成无高压示波器验证                            | 频率、互补关系、死区、启动停止波形通过   |
| v0.7 | D2 阶段验收和文档冻结                           | 满足全部验收项，允许进入 D3              |

---

## 11. 测试和验证方案

### 11.1 测试前置条件

```text
不接高压母线
不连接电机负载
使用示波器或逻辑分析仪
Gate 驱动具备可控断电/禁能手段
确认探头接地和测量范围安全
```

### 11.2 软件测试

| 编号     | 测试项目   | 通过条件                                       |
| -------- | ---------- | ---------------------------------------------- |
| D2-SW-01 | 初始化     | 状态为`SAFE_OFF`，Gate/SD 为关闭电平         |
| D2-SW-02 | Ready      | 无故障时进入`READY`                          |
| D2-SW-03 | 非法启动   | 故障锁存时启动返回`POWER_STAGE_FAULT_ACTIVE` |
| D2-SW-04 | 占空比边界 | 0、ARR 和边界值行为符合定义，非法值被拒绝      |
| D2-SW-05 | 运行更新   | 仅`RUNNING` 状态允许更新                     |
| D2-SW-06 | 正常停止   | PWM 六路关闭后 Gate/SD 关闭                    |
| D2-SW-07 | Break 触发 | 输出硬件关闭，软件故障置位并锁存               |
| D2-SW-08 | 故障恢复   | 未满足恢复条件时不能清除或重启                 |
| D2-SW-09 | 初始化异常 | 任何 HAL 失败均进入安全关闭                    |

### 11.3 示波器测试

| 编号     | 测试项目   | 通过条件                                              |
| -------- | ---------- | ----------------------------------------------------- |
| D2-HW-01 | PWM 频率   | 三相主输出频率为 10 kHz，误差符合测试设备和项目限值   |
| D2-HW-02 | 中心对齐   | 主输出上、下沿关于周期中心对称                        |
| D2-HW-03 | 互补关系   | 每一相主输出和互补输出逻辑互补                        |
| D2-HW-04 | Dead Time  | 每一相换相时不存在上下桥直通，死区达到设计值          |
| D2-HW-05 | Gate 时序  | Gate 开启前无非预期 PWM；停止时先关闭 PWM 再关闭 Gate |
| D2-HW-06 | Break 关断 | BKIN 触发后六路输出进入安全关闭状态                   |
| D2-HW-07 | 默认电平   | 复位、初始化、停止和故障状态无非预期脉冲              |

### 11.4 D2 验收证据

必须形成以下记录：

```text
编译日志和固件版本
TIM1/PWM 配置快照
启动和停止波形截图
Dead Time 测量结果
BKIN 触发波形和关断结果
Gate/SD 电平记录
异常和故障清除测试记录
测试设备、探头和测试条件
```

---

## 12. D2 阶段验收标准

以下条件全部满足后，D2 才能从“部分完成”更新为“已完成（基础项已验证）”：

```text
[ ] power_stage 模块独立建立并纳入 CMake
[ ] 初始化后 PWM 和 Gate/SD 均处于安全关闭
[ ] 六路 PWM 可通过统一接口启动和停止
[ ] 运行时三相占空比可通过统一接口更新
[ ] 占空比边界和状态约束已验证
[ ] PWM 启动顺序和停止顺序已验证
[ ] Gate/SD 独立控制接口已实现
[ ] TIM1 BKIN 硬件关断已验证
[ ] Break 软件故障记录已实现
[ ] Break 故障锁存已实现
[ ] Break 后禁止自动重启已验证
[ ] 故障清除条件和受控恢复接口已实现
[ ] PWM 频率和中心对齐行为已验证
[ ] 六路互补关系已验证
[ ] Dead Time 实际值已测量并记录
[ ] 复位、初始化、停止和故障状态无非预期脉冲
[ ] 全部测试在不接高压母线条件下完成
[ ] D2 测试记录已归档
```

任何一项未满足，D2 仍为“部分完成”，不得进入 D3 正式采样联调。

---

## 13. 后续阶段交付物

D2 完成后向 D3～D5 交付：

| 交付物                       | 使用阶段                 |
| ---------------------------- | ------------------------ |
| `PowerStage_Init()`        | D3～D9                   |
| `PowerStage_Start/Stop()`  | D3、D5、D9               |
| `PowerStage_SetDuty()`     | FOC 接入前的底层接口冻结 |
| Gate/SD 安全控制接口         | D5、D9                   |
| Break 故障状态和锁存接口     | D5、D9                   |
| PWM 频率、计数和死区测试记录 | D3、D9                   |
| 启动、停止和故障波形记录     | D5、D9                   |

D3 只能使用 D2 已冻结的 PWM 启停、输出状态和安全停止接口，不得直接操作 TIM1 CCR、CCER 或 Gate GPIO。

---

## 14. 版本与变更控制

```text
v0.1：D2 完善落地方案
v0.2：建立 PowerStage 模块和接口
v0.3：完成 Gate/SD 和安全停止
v0.4：完成 PWM 启动及占空比更新
v0.5：完成 Break 故障锁存和受控清除
v0.6：完成无高压板级波形验证
v0.7：D2 阶段验收和冻结
```

D2 冻结后，未经变更记录不得修改：

- TIM1 计数模式和 PWM 频率；
- TIM1 RepetitionCounter；
- PWM 引脚映射；
- 主输出和互补输出极性；
- Dead Time 参数；
- Break 有效电平和关断策略；
- Gate/SD 有效电平；
- PWM 启动、停止和故障恢复顺序；
- 占空比接口单位和边界定义。
