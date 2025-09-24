#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"

volatile int32_t count = 0;
float speed_pulse_ms = 0;
uint8_t flag = 0;
extern TIM_HandleTypeDef htim3;
void Encoder_Task(void const * argument)
{
    int32_t last_count = 0, delta = 0;
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    for(;;)
    {
        count = __HAL_TIM_GET_COUNTER(&htim3);
        delta = (int32_t)(count - last_count);

        if(delta > (int32_t)(htim3.Init.Period/2))
            delta -= (htim3.Init.Period + 1);
        else if(delta < -(int32_t)(htim3.Init.Period/2))
            delta += (htim3.Init.Period + 1);

        speed_pulse_ms = (float)delta / 100.0f;
        flag = 1;
        last_count = count;

//        int speed_x100 = (int)(speed_pulse_ms * 100);
//        //printf("Encoder: %ld; Speed: %d.%02d pulse/ms\r\n",
//               //count, speed_x100/100, abs(speed_x100%100));

        osDelay(500);
    }
}
