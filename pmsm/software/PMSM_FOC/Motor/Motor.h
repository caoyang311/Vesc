#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_MODE_VF = 0U,
    MOTOR_MODE_IF,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_IDENTIFICATION,
    MOTOR_MODE_COUNT
} Motor_ModeType;

void Motor_Init(void);
void Motor_SetMode(Motor_ModeType Mode);
Motor_ModeType Motor_GetMode(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_MainFunction(void);
bool Motor_GetIsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
