#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

void Servo_Init(TIM_HandleTypeDef *htim);
void Servo_SetAngle(uint16_t angle);

#endif
