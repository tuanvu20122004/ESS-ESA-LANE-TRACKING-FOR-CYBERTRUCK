#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

// ====== Thông số xe ======
#define M_PI 3.14159265358979323846
#define WHEEL_RADIUS_M   (0.065f / 2.0f)        // 0.0325 m bán kính bánh xe
#define GEAR_RATIO       (13.0f / 38.0f)        // ~0.342 -> motor quay 1 vòng, bánh xe quay 0.342 vòng
#define PULSES_PER_REV   (11.0f * 4.0f * 19.0f) // 836 xung/vòng motor
#define SAMPLE_MS        500                     // 10 ms
#define SAMPLE_TIME_S    (SAMPLE_MS / 1000.0f)



float pulses_to_mps(int pulses);
// ====== Task ======
void Encoder_Task(void const * argument);

#endif
