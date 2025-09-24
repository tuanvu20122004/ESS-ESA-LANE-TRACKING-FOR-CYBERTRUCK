#include "pid.h"

void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_Handle_t *pid, float measurement, float dt)
{
    float error = pid->setpoint - measurement;

    // Thành phần tích phân
    pid->integral += error * dt;

    // Anti-windup: giới hạn tích phân
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    // Thành phần đạo hàm
    float derivative = (error - pid->prev_error) / dt;

    // Tính đầu ra PID
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Giới hạn đầu ra
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    // Lưu sai số trước đó
    pid->prev_error = error;

    return output;
}
