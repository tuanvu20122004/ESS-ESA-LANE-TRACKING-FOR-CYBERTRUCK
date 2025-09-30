// pid.c
/*#include "pid.h"

void PID_Init(PID_Handle_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float i_min, float i_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->setpoint     = 0.0f;
    pid->integral     = 0.0f;
    pid->prev_measure = 0.0f;

    // Giới hạn đầu ra
    pid->out_min = (out_min < out_max) ? out_min : out_max;
    pid->out_max = (out_max > out_min) ? out_max : out_min;

    // Giới hạn integral riêng
    // (nên chọn sao cho Ki * i_max không vượt quá biên output quá nhiều)
    pid->i_min = (i_min < i_max) ? i_min : i_max;
    pid->i_max = (i_max > i_min) ? i_max : i_min;
}

float PID_Update(PID_Handle_t *pid, float measurement, float dt)
{
    // Bảo vệ dt
    if (dt <= 0.0f) dt = 1e-3f;

    // Sai số hiện tại
    float error = pid->setpoint - measurement;

    // Derivative trên measurement để tránh setpoint kick
    float d_meas = (measurement - pid->prev_measure) / dt;

    // Thành phần P/D
    float P = pid->Kp * error;
    float D = -pid->Kd * d_meas;

    // Tính thử integral candidate (kẹp theo biên riêng)
    float I_candidate = pid->integral + error * dt;
    if (I_candidate > pid->i_max) I_candidate = pid->i_max;
    if (I_candidate < pid->i_min) I_candidate = pid->i_min;

    // Ước lượng output nếu nhận I_candidate
    float output_est = P + pid->Ki * I_candidate + D;

    // Conditional integration:
    // - Nếu output_est nằm trong biên → nhận integral mới
    // - Nếu vượt biên, chỉ tích phân khi nó giúp kéo output về trong biên
    int within = (output_est >= pid->out_min && output_est <= pid->out_max);
    int helps =
        (output_est > pid->out_max && error < 0.0f) ||
        (output_est < pid->out_min && error > 0.0f);

    if (within || helps) {
        pid->integral = I_candidate;
    }
    // Tính output thật với integral đã chốt
    float output = P + pid->Ki * pid->integral + D;

    // Kẹp đầu ra
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    // Lưu measurement cho lần sau
    pid->prev_measure = measurement;

    return output;
}*/
#include "pid.h"

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void PID_Init(PID_Handle_t *pid,
              float Kp, float Ki, float Kd, float Kf,
              float out_min, float out_max,
              float i_min, float i_max)
{
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd; pid->Kf = Kf;

    if (out_min > out_max) { float t = out_min; out_min = out_max; out_max = t; }
    if (i_min   > i_max)   { float t = i_min;   i_min   = i_max;   i_max   = t; }

    pid->out_min = out_min; pid->out_max = out_max;
    pid->i_min   = i_min;   pid->i_max   = i_max;

    pid->setpoint     = 0.0f;
    pid->integral     = 0.0f;
    pid->prev_measure = 0.0f;
}

void PID_Reset(PID_Handle_t *pid) {
    pid->integral     = 0.0f;
    pid->prev_measure = 0.0f;
}

void PID_SetTunings(PID_Handle_t *pid, float Kp, float Ki, float Kd, float Kf) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd; pid->Kf = Kf;
}

void PID_SetOutputLimits(PID_Handle_t *pid, float out_min, float out_max) {
    if (out_min > out_max) { float t = out_min; out_min = out_max; out_max = t; }
    pid->out_min = out_min; pid->out_max = out_max;
}

void PID_SetIWindow(PID_Handle_t *pid, float i_min, float i_max) {
    if (i_min > i_max) { float t = i_min; i_min = i_max; i_max = t; }
    pid->i_min = i_min; pid->i_max = i_max;
}

float PID_Update(PID_Handle_t *pid, float measurement, float dt)
{
    if (dt <= 0.0f) dt = 1e-3f;

    // Sai số
    float error = pid->setpoint - measurement;

    // Derivative trên measurement (giảm setpoint kick)
    float d_meas = (measurement - pid->prev_measure) / dt;

    // Thành phần P, D, FF
    float P  = pid->Kp * error;
    float D  = -pid->Kd * d_meas;
    float FF =  pid->Kf * pid->setpoint;

    // Integral candidate + kẹp theo biên riêng
    float I_cand = pid->integral + error * dt;
    I_cand = clampf(I_cand, pid->i_min, pid->i_max);

    // Ước lượng output nếu nhận integral mới
    float out_est = P + pid->Ki * I_cand + D + FF;

    // Conditional integration:
    // - Nếu out_est trong [out_min..out_max] -> nhận I_cand
    // - Nếu bão hòa, chỉ tích phân khi nó giúp kéo output về trong biên
    int within = (out_est >= pid->out_min && out_est <= pid->out_max);
    int helps  =
        (out_est > pid->out_max && error < 0.0f) ||
        (out_est < pid->out_min && error > 0.0f);

    if (within || helps) {
        pid->integral = I_cand;
    }

    // Output thật + kẹp biên
    float output = P + pid->Ki * pid->integral + D + FF;
    output = clampf(output, pid->out_min, pid->out_max);

    // Lưu lại measurement
    pid->prev_measure = measurement;

    return output;
}

