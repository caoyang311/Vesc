#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_IF_ANGLE_SOURCE_MANUAL = 0U,
    MOTOR_IF_ANGLE_SOURCE_SENSOR
} Motor_IFAngleSourceType;

void Motor_IF_Init(void);
void Motor_IF_SetAngleSource(Motor_IFAngleSourceType angle_source);
void Motor_IF_ControlFunction(void);
void Motor_IF_MainFunction(void);

#endif /* MOTOR_IF_H */
