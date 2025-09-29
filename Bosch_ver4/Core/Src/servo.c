#include "servo.h"


//extern TIM_HandleTypeDef htim2;

TIM_HandleTypeDef *htim2_t;
void servo_init(TIM_HandleTypeDef *htim)
{
	htim2_t = htim;
}

void Servo_SetAngle(uint16_t angle)
{
    if(angle > SERVO_RANGE) angle = SERVO_RANGE;
    if(angle < -SERVO_RANGE) angle = -SERVO_RANGE;

    uint16_t servoResolution = (2450 - 450)/180;
    uint16_t value = (uint16_t)(servoResolution * angle + 530);

    __HAL_TIM_SET_COMPARE(htim2_t,TIM_CHANNEL_1,value);
}
