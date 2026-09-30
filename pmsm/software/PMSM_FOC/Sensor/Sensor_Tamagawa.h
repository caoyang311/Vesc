#ifndef SENSOR_TAMAGAWA_H
#define SENSOR_TAMAGAWA_H

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_TAMAGAWA_READ_COMMAND       (0x02U) /**< 读取角度命令 */
#define SENSOR_TAMAGAWA_RESPONSE_LENGTH     (6U) /**< 响应长度 */
#define SENSOR_TAMAGAWA_RESOLUTION          (131072U) /**< 分辨率 */
#define SENSOR_TAMAGAWA_COUNT_MASK         (SENSOR_TAMAGAWA_RESOLUTION - 1U) /**< 有效位掩码 */
#define SENSOR_TAMAGAWA_TWO_PI             (6.28318530717958647692F) /**< 2π */

void Sensor_Tamagawa_Init(void);
void Sensor_Tamagawa_ProcessAngle(void);
void Sensor_Tamagawa_RequestAngle(void);
uint32_t Sensor_Tamagawa_GetRawCount(void);
float Sensor_Tamagawa_GetMechanicalAngleRad(void);
float Sensor_Tamagawa_GetElectricalAngleRad(uint8_t pole_pairs);
uint16_t Sensor_Tamagawa_GetElectricalPhase(uint8_t pole_pairs);
bool Sensor_Tamagawa_IsDataValid(void);
void Sensor_Tamagawa_ClearDataValid(void);

#endif /* SENSOR_TAMAGAWA_H */
