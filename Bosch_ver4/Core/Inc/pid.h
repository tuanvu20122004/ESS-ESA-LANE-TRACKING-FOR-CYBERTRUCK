#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;     // giá trị mong muốn
    float integral;     // thành phần tích phân
    float prev_error;   // lưu sai số trước đó

    float out_min;      // giới hạn min đầu ra
    float out_max;      // giới hạn max đầu ra
} PID_Handle_t;

// Khởi tạo PID
void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

// Cập nhật PID, trả về đầu ra
float PID_Update(PID_Handle_t *pid, float measurement, float dt);

#endif
