#include "encoder.h"
#include "cmsis_os.h"
#include "math.h"
#include "pid.h"
#include "communication.h" // DEBUG

volatile uint8_t flag = 0;
float car_speed_mps = 0.0f;            // Tốc độ xe (m/s)
extern int motorSpeed;                // PWM output cho motor
uint16_t counterAfter = 0, counterInitial = 0, delta = 0;

//extern TIM_HandleTypeDef htim3;
//extern PID_Handle_t pid;               // PID khai báo ở freertos_tasks.c

TIM_HandleTypeDef* htim3_t;
PID_Handle_t* pid_encoder;               // PID khai báo ở freertos_tasks.c

// hàm init

void encoder_init(TIM_HandleTypeDef* htim, PID_Handle_t* pid)
{
	htim3_t = htim;
	pid_encoder = pid;
}


// Hàm chuyển đổi xung thành tốc độ tuyến tính (m/s)
float pulses_to_mps(int pulses) {
    float radius = 0.065f / 2.0f;  // Bán kính bánh xe (m)
    float gear_ratio = 13.0f / 38.0f;  // Tỷ số bánh răng
    float pulses_per_rev = 11.0f * 4.0f * 19.0f;  // Số xung mỗi vòng quay
    float time_interval = 0.01f;  // Thời gian mỗi chu kỳ (s)

    float wheel_rps = ((pulses / pulses_per_rev) / time_interval) * gear_ratio;
    float speed = wheel_rps * (2.0f * M_PI * radius);

    return speed;
}

void Encoder_Task(void const * argument)
{
    // Khởi động encoder
    HAL_TIM_Encoder_Start(htim3_t, TIM_CHANNEL_ALL);
    //TickType_t lastWakeTime = osKernelSysTick();

    for(;;)
    	{
    		  counterAfter = __HAL_TIM_GET_COUNTER(htim3_t);
    		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim3_t))
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

        car_speed_mps = pulses_to_mps(delta);
        // ====== PID Control ======
        float dt = SAMPLE_TIME_S;
        float output = PID_Update(pid_encoder, car_speed_mps, dt);
        motorSpeed = (int)output;  // Lưu lại để Motor_Servo_Task dùng

        // Gửi feedback trực tiếp về Pi
        Send_SpeedFeedback(pid_encoder->setpoint, car_speed_mps, motorSpeed);  // DEBUG

        // Báo có dữ liệu mới
        flag = 1;
        counterInitial = counterAfter;

        // Delay đúng chu kỳ
        osDelay(10);
    }
}
