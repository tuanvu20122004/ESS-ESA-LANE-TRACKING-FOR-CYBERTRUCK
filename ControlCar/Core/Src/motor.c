#include "motor.h"

static TIM_HandleTypeDef *motor_htim;

void Motor_Init(TIM_HandleTypeDef *htim) {
    motor_htim = htim;
    HAL_TIM_PWM_Start(motor_htim, TIM_CHANNEL_1);
}

void Motor_SetSpeed(uint16_t speed, uint8_t direction) {
    if (speed > 255) speed = 255;

    float motorResolution = (719.0) / 255;
    uint16_t value = motorResolution * speed;
    motor_htim->Instance->CCR1 = value;

    if (direction == 1) { // forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else { // backward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
