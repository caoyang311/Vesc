# 编码器位置与速度计算方案设计

## 1. 文档目的

本文档定义基于 TIM2 增量式编码器接口的电机位置、速度及方向计算方案，供代码实施前审核。

本阶段仅形成方案，不修改现有源文件，不实现代码。PA2 Z 相相关功能暂不纳入本阶段范围。

## 2. 已确认需求

模块需要提供：

1. 电机电角度；
2. 电角速度；
3. 机械转速；
4. 电机旋转方向；
5. TIM2 编码器计数启动与读取。

方向定义：从电机输出轴方向观察，逆时针为正转，顺时针为反转。

## 3. 已确认参数与硬件配置

### 3.1 TIM2 编码器接口

当前工程配置为：

- TIM2_CH1：PA0，编码器 A 相；
- TIM2_CH2：PA1，编码器 B 相；
- 编码器模式：`TIM_ENCODERMODE_TI12`；
- A、B 两相均为上升沿极性；
- 输入滤波：10；
- 预分频：0；
- 向上计数模式；
- 自动重装载值：`0xFFFFFFFF`；
- TIM2 为 32 位自由运行计数器。

TI12 编码器模式对 A/B 两相信号进行四倍频计数。

### 3.2 电机和编码器参数

已确认参数：

- 电机磁极数：8；
- 电机极对数：4；
- 编码器分辨率：1000 PPR；
- 四倍频后每机械转计数：4000 count/rev；
- 机械速度计算周期：5 ms；
- 暂不使用脉冲周期测速法；
- 暂不使用 Z 相校准、同步或诊断功能。

```text
counts_per_revolution = encoder_ppr × 4
                      = 1000 × 4
                      = 4000 count/rev
```

实际 A/B 接线已经确认：从输出轴方向观察，逆时针旋转时 TIM2 计数增加。因此本阶段不需要软件反向修正。

## 4. 软件架构方案

### 4.1 模块位置

建议在 `Platform` 层新增：

```text
Platform/EncoderService.c
Platform/EncoderService.h
```

原因：

- TIM2 和 HAL 句柄属于硬件能力；
- App 和 FOC 层不应直接访问 HAL或 TIM2 寄存器；
- 公共头文件不暴露 `TIM_HandleTypeDef` 等 HAL 类型；
- 模块向上层提供经过换算的机械和电气量。

### 4.2 依赖方向

```text
MotorControl / FOC快速控制
            |
            v
     EncoderService.h
            |
            v
       TIM2 HAL / Core配置
```

TIM2 启动和计数器读取全部由 `EncoderService.c` 封装。

本阶段不修改 DIO 配置，也不读取 PA2 Z 相。

## 5. 建议公共接口

### 5.1 配置类型

```c
typedef struct
{
    uint32_t counts_per_revolution;
    uint16_t pole_pairs;
    float speed_sample_period_s;
    uint32_t initial_count;
} EncoderService_ConfigType;
```

字段说明：

- `counts_per_revolution`：四倍频后的每机械转计数，固定为 4000；
- `pole_pairs`：电机极对数，固定为 4；
- `speed_sample_period_s`：机械速度计算周期，固定为 0.005 s；
- `initial_count`：软件机械位置参考计数，默认取启动时 TIM2 计数。

暂不设置 `electrical_zero_phase`。本阶段电角度以编码器服务启动位置作为相对机械零点，后续结合预定位和 Z 相方案再增加绝对电角零点标定。

### 5.2 方向类型

```c
typedef enum
{
    ENCODER_DIRECTION_STOP = 0U,
    ENCODER_DIRECTION_COUNTERCLOCKWISE,
    ENCODER_DIRECTION_CLOCKWISE
} EncoderService_DirectionType;
```

语义：

- `COUNTERCLOCKWISE`：从输出轴观察为逆时针，正转，速度为正；
- `CLOCKWISE`：从输出轴观察为顺时针，反转，速度为负；
- `STOP`：5 ms 采样周期内无有效计数变化。

### 5.3 输出类型

```c
typedef struct
{
    uint32_t raw_count;
    int32_t delta_count;
    uint32_t mechanical_count;
    uint16_t mechanical_angle_phase;
    uint16_t electrical_angle_phase;
    float electrical_speed_rad_s;
    float mechanical_speed_rpm;
    EncoderService_DirectionType direction;
} EncoderService_OutputType;
```

相位约定：

```text
0x0000 -> 0°
0x4000 -> 90°
0x8000 -> 180°
0xC000 -> 270°
0xFFFF -> 接近360°
```

电角度相位可直接输入当前 `FocTrig_GetSinCos()` 接口。

### 5.4 函数接口

```c
bool EncoderService_Init(const EncoderService_ConfigType * config);
bool EncoderService_Start(void);
void EncoderService_UpdatePosition(void);
void EncoderService_UpdateSpeed(void);
bool EncoderService_GetOutput(EncoderService_OutputType * output);
```

职责：

- `Init`：检查配置，读取或设置初始参考计数并清零内部状态；
- `Start`：调用 `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)`；
- `UpdatePosition`：读取 TIM2 并更新机械角度和电角度；
- `UpdateSpeed`：每 5 ms 计算计数差、机械转速、电角速度和方向；
- `GetOutput`：向上层提供一致的结果快照。

## 6. 32 位计数回绕处理

TIM2 的 ARR 为 `0xFFFFFFFF`，正转和反转时均可能跨越计数边界。

计数差采用模 2^32 运算：

```text
unsigned_delta = current_count - previous_count
delta_count = 将 unsigned_delta 的32位位模式解释为有符号差值
```

目标数学结果等价于：

```text
delta_count = (int32_t)(current_count - previous_count)
```

该方法自动处理：

- 正方向：`0xFFFFFFFF -> 0x00000000`；
- 负方向：`0x00000000 -> 0xFFFFFFFF`。

前提是单个 5 ms 周期内计数变化绝对值小于 `2^31`。

在额定转速 3000 RPM 时：

```text
3000 RPM = 50 rev/s
count_rate = 50 × 4000 = 200000 count/s
delta_count_5ms = 200000 × 0.005 = 1000 count
```

远小于 `2^31`，满足回绕差分前提。

注意：无符号转有符号且数值不可表示时属于实现定义行为。实施时需结合当前 ARM GCC 二补码目标进行封装，并记录对应 MISRA 偏差；也可以通过显式分支计算正负差值，避免依赖实现定义转换。

## 7. 机械位置计算

以编码器服务启动位置为相对机械零点：

```text
relative_count = current_count - initial_count
```

将相对计数归一化到一机械转：

```text
mechanical_count = relative_count mod counts_per_revolution
```

范围：

```text
0 <= mechanical_count < 4000
```

机械角度相位：

```text
mechanical_angle_phase
    = mechanical_count × 65536 / 4000
```

每个编码器计数对应：

```text
360° / 4000 = 0.09° mechanical
```

中间乘法使用 `uint64_t`，避免配置扩展后发生整数溢出。

## 8. 电角度计算

电角度与机械角度关系：

```text
electrical_angle = mechanical_angle × pole_pairs
```

当前极对数为 4：

```text
electrical_count
    = (mechanical_count × 4) mod 4000

electrical_angle_phase
    = electrical_count × 65536 / 4000
```

一机械转包含 4 个电周期，`electrical_angle_phase` 每 1000 个编码器计数回绕一次。

当前输出为相对于模块启动位置的电角度，不作为已标定的绝对转子电角度。角度闭环启用前仍需要后续确定绝对电角零点方案。

## 9. 机械转速计算

采用固定 5 ms 周期的 M 法测速，不增加脉冲周期法：

```text
mechanical_speed_rpm
    = delta_count × 60
      / (counts_per_revolution × sample_period_s)
```

代入当前参数：

```text
mechanical_speed_rpm
    = delta_count × 60 / (4000 × 0.005)
    = delta_count × 3 RPM
```

因此原始机械转速分辨率为：

```text
3 RPM/count
```

额定转速 3000 RPM 对应：

```text
delta_count = 1000 count/5ms
```

可使用现有窗口长度为 10 的滑动平均滤波器对机械转速进行滤波：

- 更新周期：5 ms；
- 窗口长度：10；
- 滤波窗口覆盖时间：50 ms。

是否默认启用滤波可在代码实施时作为固定配置处理。

## 10. 电角速度计算

机械角速度：

```text
mechanical_speed_rad_s
    = mechanical_speed_rpm × 2π / 60
```

电角速度：

```text
electrical_speed_rad_s
    = mechanical_speed_rad_s × pole_pairs
```

合并为：

```text
electrical_speed_rad_s
    = mechanical_speed_rpm × 4 × 2π / 60
```

也可直接由 5 ms 计数差计算：

```text
electrical_speed_rad_s
    = delta_count × 2π × pole_pairs
      / (counts_per_revolution × sample_period_s)
```

速度保留符号：

- 逆时针为正；
- 顺时针为负；
- 停止为 0。

## 11. 旋转方向判定

根据 5 ms 周期内的净计数差判定：

```text
delta_count > 0 -> 逆时针，正转
delta_count < 0 -> 顺时针，反转
delta_count = 0 -> 停止
```

已确认逆时针时 TIM2 计数增加，因此不增加软件方向翻转配置。

为避免零速附近编码器抖动导致方向频繁切换，可以设置计数死区：

```text
abs(delta_count) <= direction_deadband_count -> STOP
```

初始建议死区为 0 count，若目标机测试发现静止抖动，再调整为 1 count。

TIM2 `DIR` 位仅作为诊断辅助，不作为最终方向输出依据，因为瞬时方向位不能反映完整 5 ms 周期内的净位移。

## 12. 调度与执行域

### 12.1 初始化

建议系统初始化顺序：

```text
MX_TIM2_Init
  -> EncoderService_Init
  -> EncoderService_Start
```

`EncoderService_Start()` 失败时，`System_Init()` 返回失败，不进入正常电机控制。

### 12.2 快速位置更新

电角度供 FOC 使用，建议在 TIM1 10 kHz 快速控制入口调用：

```text
EncoderService_UpdatePosition()
FOC读取 electrical_angle_phase
```

快速函数仅执行：

- 一次 TIM2 计数快照；
- 有界整数角度计算；
- 更新内部位置结果。

不得阻塞、不得使用动态内存、不得执行速度滤波。

### 12.3 速度更新

在现有 5 ms 协作任务中调用：

```text
EncoderService_UpdateSpeed()
```

该函数计算：

- 5 ms 计数差；
- 机械转速；
- 电角速度；
- 旋转方向；
- 可选的 10 点滑动平均速度。

### 12.4 数据一致性

TIM1 ISR 更新位置，而 5 ms 主循环任务更新速度并读取输出，需要避免多字段撕裂。

推荐方案：

1. 快速位置更新和 5 ms 速度更新分别读取一次 TIM2；
2. `GetOutput()` 使用短临界区复制结果快照；
3. 临界区只覆盖结构体复制，不覆盖浮点计算或 HAL 调用；
4. 明确 TIM1 ISR 和主循环对上下文字段的所有权。

## 13. 数据有效性和错误处理

建议状态类型：

```c
typedef enum
{
    ENCODER_STATUS_UNINITIALIZED = 0U,
    ENCODER_STATUS_RUNNING,
    ENCODER_STATUS_CONFIG_ERROR,
    ENCODER_STATUS_HARDWARE_ERROR
} EncoderService_StatusType;
```

参数检查：

- `counts_per_revolution > 0`；
- `pole_pairs > 0`；
- `speed_sample_period_s > 0.0F`；
- 指针参数不得为 `NULL`；
- 角度中间乘法使用 `uint64_t`；
- 未初始化或启动失败时不返回伪造的有效角度和速度；
- HAL 启动失败必须反馈给 `System_Init()`；
- 快速位置接口执行时间必须有界。

## 14. Z 相延期范围

本阶段明确不实现：

- PA2 Z 相读取；
- Z 相边沿检测；
- Z 相机械零点同步；
- Z 相电角零点偏移；
- Z 相丢失或周期一致性诊断；
- Z 相外部中断。

因此本阶段不修改：

```text
Platform/Dio.h
Platform/Dio_Config.h
Platform/Dio.c
```

后续引入 Z 相时，应单独形成标定与同步方案，并保证同步动作不会造成速度差分突变。

## 15. C Safety Coding Standard要求

实施时遵循：

- 公共头文件不包含 `tim.h` 或 HAL 类型；
- HAL 依赖仅存在于 `EncoderService.c`；
- 所有函数声明和定义均添加 Doxygen；
- 使用固定宽度整数和 `bool`；
- 所有局部变量显式初始化；
- 外部指针参数执行空指针检查；
- 不使用动态内存、递归和无界循环；
- ISR 路径保持有界、非阻塞；
- 有符号和无符号转换明确；
- `INT32_MIN` 等边界不得直接取负；
- 返回值由调用方检查；
- 共享数据采用明确的一致性策略；
- App 和 FOC 层不直接访问 TIM2；
- 对二补码转换依赖进行静态分析检查，必要时记录局部 MISRA 偏差。

## 16. 测试方案

### 16.1 单元测试

至少覆盖：

1. 静止：`delta_count = 0`；
2. 逆时针正转：正计数差；
3. 顺时针反转：负计数差；
4. 正向跨越 `0xFFFFFFFF -> 0`；
5. 反向跨越 `0 -> 0xFFFFFFFF`；
6. 机械角度在 0、1/4、1/2、3/4 和接近一圈位置；
7. 四个电周期的角度回绕；
8. 无效配置和空指针；
9. 额定转速 3000 RPM，对应 1000 count/5ms；
10. 低速 1 count/5ms，对应 3 RPM；
11. 速度滤波启动阶段和窗口填满阶段；
12. 方向死区边界；
13. 模块未初始化和硬件启动失败。

### 16.2 目标机测试

需要验证：

- 从输出轴观察，手动逆时针转动时速度为正、方向为正转；
- 从输出轴观察，手动顺时针转动时速度为负、方向为反转；
- 一机械转 TIM2 计数增加或减少 4000；
- 3000 RPM 下每 5 ms 约为 1000 count；
- 机械转速误差；
- 电角速度误差；
- 四个电周期的电角度回绕；
- 10 kHz 位置更新 WCET；
- 5 ms 速度更新周期抖动；
- 静止时方向和速度输出稳定性；
- 编码器 A/B 断线或单相信号丢失行为。

## 17. 实施阶段预计修改范围

审核通过后预计：

### 新增

```text
Platform/EncoderService.c
Platform/EncoderService.h
```

### 修改

```text
System/System.c
System/TaskConfig.c
Platform/ServiceCallback.c
CMakeLists.txt
```

可能由 `MotorControl` 增加编码器结果读取，但不得直接读取 TIM2。

## 18. 已冻结实施条件

当前实施条件已确认：

1. 编码器为 1000 PPR，四倍频后 4000 count/rev；
2. 从电机输出轴方向观察定义旋转方向；
3. TIM2 计数增加对应逆时针和正转；
4. 本阶段不考虑 Z 相；
5. 机械速度固定采用 5 ms 计算周期；
6. 本阶段只采用定周期 M 法测速，不增加脉冲周期法。
