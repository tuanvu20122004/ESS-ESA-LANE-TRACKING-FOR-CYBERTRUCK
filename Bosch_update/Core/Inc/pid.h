// pid.h
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Gains
    float Kp;
    float Ki;
    float Kd;
    float Kf;          // feedforward: output ≈ Kf * setpoint (cùng thang với output)

    // State
    float setpoint;    // đơn vị giống measurement (vd: m/s)
    float integral;    // tích phân (đơn vị "đầu vào" của Ki)
    float prev_measure;// y(k-1) để tính derivative trên measurement

    // Limits
    float out_min;     // giới hạn đầu ra
    float out_max;
    float i_min;       // giới hạn tích phân (riêng)
    float i_max;
} PID_Handle_t;

/** Khởi tạo PID đầy đủ */
void PID_Init(PID_Handle_t *pid,
              float Kp, float Ki, float Kd, float Kf,
              float out_min, float out_max,
              float i_min, float i_max);

/** Đặt lại trạng thái (integral/prev_measure) */
void PID_Reset(PID_Handle_t *pid);

/** Cập nhật thông số */
void PID_SetTunings(PID_Handle_t *pid, float Kp, float Ki, float Kd, float Kf);

/** Cập nhật giới hạn đầu ra */
void PID_SetOutputLimits(PID_Handle_t *pid, float out_min, float out_max);

/** Cập nhật giới hạn tích phân riêng */
void PID_SetIWindow(PID_Handle_t *pid, float i_min, float i_max);

/** Đặt setpoint (đơn vị như measurement) */
static inline void PID_SetSetpoint(PID_Handle_t *pid, float sp) { pid->setpoint = sp; }

/** Tính điều khiển: measurement (vd m/s), dt (s). Trả về output đã kẹp trong [out_min..out_max] */
float PID_Update(PID_Handle_t *pid, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */

