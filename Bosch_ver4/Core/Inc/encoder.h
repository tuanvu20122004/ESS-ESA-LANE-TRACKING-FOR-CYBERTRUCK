#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "pid.h"


// ====== Thông số xe ======
#define M_PI 3.14159265358979323846
#define WHEEL_RADIUS_M   (0.065f / 2.0f)        // 0.0325 m bán kính bánh xe
#define GEAR_RATIO       (13.0f / 38.0f)        // ~0.342 -> motor quay 1 vòng, bánh xe quay 0.342 vòng
#define PULSES_PER_REV   (11.0f * 4.0f * 19.0f) // 836 xung/vòng motor
#define SAMPLE_MS        500                     // 10 ms
#define SAMPLE_TIME_S    (SAMPLE_MS / 1000.0f)


// ====== Biến toàn cục ======
//extern volatile int32_t count;      // giá trị counter của encoder
//extern volatile float speed_pulse_s; // xung/giây
//extern volatile uint8_t flag;        // cờ báo có dữ liệu mới
//extern float car_speed_mps;          // tốc độ xe (m/s)
//extern int motorSpeed;               // output PWM từ PID

// ====== Task ======

void encoder_init(TIM_HandleTypeDef* htim, PID_Handle_t* pid);

float pulse_to_mps(int pulse);
void Encoder_Task(void const * argument);

#endif
