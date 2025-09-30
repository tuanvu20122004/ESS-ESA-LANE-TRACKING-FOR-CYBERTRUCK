// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "pid.h"
#include "communication.h"
// ====== Thông số cơ khí/encoder ======
#define M_PI             3.14159265358979323846
#define WHEEL_RADIUS_M   (0.065f / 2.0f)             // 0.0325 m
#define GEAR_RATIO       (13.0f / 38.0f)             // motor->wheel ≈ 0.342
#define PULSES_PER_REV   (11.0f * 4.0f * 19.0f)      // 836 xung/vòng motor

// ====== Thời gian mẫu (dùng đồng nhất cho đo tốc độ & PID) ======
#define SAMPLE_MS        (10)                         // 10 ms
#define SAMPLE_TIME_S    (SAMPLE_MS / 1000.0f)

// ====== API ======
float pulses_to_mps(int pulses);
void Encoder_Task(void const *argument);

#endif // ENCODER_H
