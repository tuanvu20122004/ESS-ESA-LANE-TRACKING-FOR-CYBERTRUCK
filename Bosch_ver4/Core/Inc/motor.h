#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#define MOTOR_MAX_INPUT 255
void motor_init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(int speed);

#endif
