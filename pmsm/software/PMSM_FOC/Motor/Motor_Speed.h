#ifndef MOTOR_SPEED_H
#define MOTOR_SPEED_H

#include <stdbool.h>
#include <stdint.h>

void Motor_Speed_Init(void);
void Motor_Speed_MainFunction(void);
void Motor_Speed_SetReferenceRpm(float reference_rpm);
void Motor_Speed_ControlFunction(void);
#endif /* MOTOR_SPEED_H */
