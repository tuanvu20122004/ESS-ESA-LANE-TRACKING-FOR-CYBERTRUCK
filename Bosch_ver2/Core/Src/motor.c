#include "motor.h"
#include <stdlib.h>
extern TIM_HandleTypeDef htim1;
void Motor_SetSpeed(int speed)
{
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    float motorResolution = (19999.0)/255;
    uint16_t value = motorResolution * abs(speed);

    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}
