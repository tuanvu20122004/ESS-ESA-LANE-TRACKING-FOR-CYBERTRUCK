#include "encoder.h"
#include "cmsis_os.h"
#include "math.h"
#include "pid.h"
#include "communication.h"// DEBUG
volatile int32_t count = 0;
volatile float speed_pulse_s = 0.0f;   // xung/giây
volatile uint8_t flag = 0;
float car_speed_mps = 0.0f;            // tốc độ xe (m/s)
extern int motorSpeed;               // PWM output cho motor

extern TIM_HandleTypeDef htim3;
extern PID_Handle_t pid;               // PID khai báo ở freertos_tasks.c

void Encoder_Task(void const * argument)
{
    int32_t last_count = 0, delta = 0;
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    TickType_t lastWakeTime = osKernelSysTick();

    for(;;)
    {
        // Đọc giá trị counter
        count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        delta = (count - last_count);

        // Xử lý tràn counter
        const int32_t halfPeriod = (int32_t)(htim3.Init.Period / 2);
        const int32_t maxCount   = (int32_t)(htim3.Init.Period + 1);

        if(delta > halfPeriod) {
            delta -= maxCount;
        } else if(delta < -halfPeriod) {
            delta += maxCount;
        }

        // ====== Tính vận tốc ======
        // pulses per second
        speed_pulse_s = ((float)delta * 1000.0f) / (float)SAMPLE_MS;

        // motor revolutions per second
        float motor_rps = speed_pulse_s / PULSES_PER_REV;

        // wheel revolutions per second (sau hộp số)
        float wheel_rps = motor_rps * GEAR_RATIO;

        // linear speed (m/s)
        car_speed_mps = wheel_rps * 2.0f * (float)M_PI * WHEEL_RADIUS_M;

        // ====== PID Control ======
        float dt = SAMPLE_TIME_S;
        float output = PID_Update(&pid, car_speed_mps, dt);
        motorSpeed = (int)output; // lưu lại để Motor_Servo_Task dùng
        // Gửi feedback trực tiếp về Pi
        Send_SpeedFeedback(car_speed_mps, pid.setpoint, motorSpeed);// DEBUG
        // Báo có dữ liệu mới
        flag = 1;
        last_count = count;

        // Delay đúng chu kỳ
        osDelayUntil(&lastWakeTime, SAMPLE_MS);
    }
}
