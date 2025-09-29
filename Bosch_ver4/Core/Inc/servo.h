#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#define SERVO_RANGE 180
void servo_init(TIM_HandleTypeDef *htim);
void Servo_SetAngle(uint16_t angle);

#endif
