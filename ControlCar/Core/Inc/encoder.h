#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

void Encoder_Init(TIM_HandleTypeDef *htim);
uint16_t Encoder_GetDelta(void);

#endif
