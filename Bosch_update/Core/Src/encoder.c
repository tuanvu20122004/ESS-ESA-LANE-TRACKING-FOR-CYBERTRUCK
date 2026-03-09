// encoder.c
#include "encoder.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim3;
extern PID_Handle_t pid;
extern int motorSpeed;

volatile uint8_t flag = 0;
float car_speed_mps = 0.0f;

float pulses_to_mps(int pulses) {
    const float rev_motor   = ((float)pulses) / PULSES_PER_REV;       // vòng motor trong Δt
    const float rps_motor   = rev_motor / SAMPLE_TIME_S;              // rps motor
    const float rps_wheel   = rps_motor * GEAR_RATIO;                 // rps bánh
    const float speed_mps   = rps_wheel * (2.0f * M_PI * WHEEL_RADIUS_M);
    return speed_mps;
}

void Encoder_Task(void const *argument)
{
    // Bật encoder & đọc mốc ban đầu
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    uint16_t counterInitial = __HAL_TIM_GET_COUNTER(&htim3);

    for (;;)
    {
        // Đọc giá trị hiện tại
        uint16_t counterAfter = __HAL_TIM_GET_COUNTER(&htim3);

        // Chênh lệch có dấu kiểu 16-bit tự xử lý wrap-around 0..65535
        int16_t diff = (int16_t)((int32_t)counterAfter - (int32_t)counterInitial);

        // Đổi xung → m/s (có dấu)
        car_speed_mps = pulses_to_mps((int)diff);

        // PID control
        float output = PID_Update(&pid, car_speed_mps, SAMPLE_TIME_S);
        motorSpeed = (uint16_t)output;

        // Feedback (tùy mục đích debug/giám sát)
        Send_SpeedFeedback(pid.setpoint, car_speed_mps);
        //Send_PWM_Feedback(motorSpeed, car_speed_mps);
        // Báo có dữ liệu mới & cập nhật mốc
        flag = 1;
        counterInitial = counterAfter;

        // Delay đúng chu kỳ
        osDelay(SAMPLE_MS);
    }
}
