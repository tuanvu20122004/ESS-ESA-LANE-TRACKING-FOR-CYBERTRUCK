#include "servo.h"
extern TIM_HandleTypeDef htim2;
void Servo_SetAngle(int angle)
{
    if(angle > SERVO_RANGE) angle = SERVO_RANGE;
    if(angle < -SERVO_RANGE) angle = -SERVO_RANGE;

    float servoResolution = (2450.0 - 450.0)/180;
    uint16_t value = servoResolution * angle + 530;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value);
}
