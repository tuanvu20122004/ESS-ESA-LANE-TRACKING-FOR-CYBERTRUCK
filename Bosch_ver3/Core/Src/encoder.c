#include "encoder.h"
#include "cmsis_os.h"
#include "math.h"
#include "pid.h"
#include "communication.h"// DEBUG

// ====== Biến toàn cục ======
extern TIM_HandleTypeDef htim3;
extern PID_Handle_t pid;               // PID khai báo ở freertos_tasks.c


uint16_t motorSpeed =  0, car_speed_mps = 0;
uint16_t counterAfter= 0, counterInitial = 0, delta= 0;
uint8_t flag = 0;

float pulses_to_mps(int pulses) {
    float radius = 0.065f / 2.0f;
    float gear_ratio = 13.0f / 38.0f;
    float pulses_per_rev = 11.0f * 4.0f * 19.0f;
    float time_interval = 0.01f;

    float wheel_rps = ((pulses / pulses_per_rev) / time_interval) * gear_ratio;
    float speed = wheel_rps * (2.0f * M_PI * radius);

    return speed;
}

void Encoder_Task(void const * argument)
{

		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
		TickType_t lastWakeTime = osKernelSysTick();

		for(;;)
		{
		  counterAfter = __HAL_TIM_GET_COUNTER(&htim3);
		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
		  {
			  if(counterAfter > counterInitial)
			  {
				  delta = 65535 - counterAfter + counterInitial;
			  }
			  else if (counterAfter <= counterInitial)
			  {
				  delta = counterInitial - counterAfter;
			  }
		  }
		  else
		  {
			  if(counterAfter >= counterInitial)
			  {
				  delta = counterAfter - counterInitial;
			  }
			  else if (counterAfter < counterInitial)
			  {
				  delta = (65535 - counterInitial) + counterAfter + 1;
			  }
		  }
        // linear speed (m/s)
        car_speed_mps = pulses_to_mps(delta);

        // ====== PID Control ======
        float dt = SAMPLE_TIME_S;
        float output = PID_Update(&pid, car_speed_mps, dt);
        motorSpeed = (int)output; // lưu lại để Motor_Servo_Task dùng
        // Gửi feedback trực tiếp về Pi
        Send_SpeedFeedback(car_speed_mps, pid.setpoint, motorSpeed);// DEBUG
        // Báo có dữ liệu mới
        flag = 1;
        counterInitial = counterAfter;
        // Delay đúng chu kỳ
        osDelayUntil(&lastWakeTime, SAMPLE_MS);
    }
}
