#ifndef SENSOR_TAMAGAWA_H
#define SENSOR_TAMAGAWA_H

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_TAMAGAWA_READ_COMMAND       (0x02U) /**< 读取角度命令 */
#define SENSOR_TAMAGAWA_RESPONSE_LENGTH     (6U) /**< 响应长度 */
#define SENSOR_TAMAGAWA_RESOLUTION          (131072U) /**< 分辨率 */
#define SENSOR_TAMAGAWA_COUNT_MASK         (SENSOR_TAMAGAWA_RESOLUTION - 1U) /**< 有效位掩码 */
#define SENSOR_TAMAGAWA_TWO_PI             (6.28318530717958647692F) /**< 2π */
#define SENSOR_TAMAGAWA_UPDATE_PERIOD_S    (0.001F)

void Sensor_Tamagawa_Init(void);
void Sensor_Tamagawa_ProcessAngle(void);
void Sensor_Tamagawa_RequestAngle(void);
void Sensor_Tamagawa_SampleSpeed(float sample_period_s,uint8_t pole_pairs);
uint16_t Sensor_Tamagawa_GetElectricalPhase(uint8_t pole_pairs);
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithOffset(uint8_t pole_pairs);
uint16_t Sensor_Tamagawa_GetElectricalPhaseWithCompensation(float compensation_time_s);
float Sensor_Tamagawa_GetElectricalAngularSpeedRadS(void);
float Sensor_Tamagawa_GetMotorSpeedRpm(uint8_t pole_pairs,float electrical_speed);
void Sensor_Tamagawa_SetElectricalPhaseOffset(uint16_t offset);
bool Sensor_Tamagawa_IsElectricalOffsetValid(void);

#endif /* SENSOR_TAMAGAWA_H */
