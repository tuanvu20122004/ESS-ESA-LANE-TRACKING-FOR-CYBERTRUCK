#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(uint16_t speed, uint8_t direction);

#endif
