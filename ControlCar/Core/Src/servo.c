#include "servo.h"

static TIM_HandleTypeDef *servo_htim;

void Servo_Init(TIM_HandleTypeDef *htim) {
    servo_htim = htim;
    HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_1);
}

void Servo_SetAngle(uint16_t angle) {
    if (angle > 180) angle = 180;
    float resolution = (2450.0 - 450.0) / 180;
    uint16_t value = resolution * angle + 530;
    servo_htim->Instance->CCR1 = value;
}
