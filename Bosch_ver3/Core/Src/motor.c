#include "motor.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;

void Motor_SetSpeed(int speed)
{
	if(speed > MOTOR_MAX_INPUT)
	{
		speed = MOTOR_MAX_INPUT;
	}
	if(speed < -MOTOR_MAX_INPUT)
	{
		speed = -MOTOR_MAX_INPUT;
	}

	uint16_t period = htim1.Init.Period;
	float motorResolution = (float)period/(float)MOTOR_MAX_INPUT;
	uint16_t value = (uint16_t)(motorResolution * abs(speed));

	if(speed > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,value);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	}
	else if(speed < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,value);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	}
}
