#include "freertos_tasks.h"

osThreadId CommunicationHandle;
osThreadId Motor_Servo_TasHandle;
osThreadId Re_Encoder_TaskHandle;

int motorSpeed = 0;   // PID output (PWM duty)
int steerAngle = 0;   // Servo angle từ Pi
PID_Handle_t pid;     // PID controller

static void Motor_Servo_Task(void const * argument)
{
    // Enable cầu H
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);

    for(;;) {
        Motor_SetSpeed(motorSpeed);   // motorSpeed do Encoder_Task + PID tính
        Servo_SetAngle(steerAngle);   // steerAngle do Communication_Task cập nhật
        osDelay(20); // 50Hz update rate
    }
}

// FreeRTOS Init
void MX_FREERTOS_Init(void)
{
    // ====== PID gains (tạm) ======
    const float Kp = 50000.0f;
    const float Ki =  2000.0f;
    const float Kd =  2000.0f;
    const float Kf =  20000.0f;

    const float i_band = (0.5f * (float)MOTOR_MAX_INPUT) / Ki;

    // PID_Init mới: (Kp, Ki, Kd, Kf, out_min, out_max, i_min, i_max)
    PID_Init(&pid, Kp, Ki, Kd, Kf, 0.0f, (float)MOTOR_MAX_INPUT,-i_band, i_band);
    // Set setpoint (m/s)
    PID_SetSetpoint(&pid, 0.3f);
    // Communication task
    osThreadDef(Communication, Communication_Task, osPriorityNormal, 0, 1024);
    CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

    // Motor + Servo task
    osThreadDef(Motor_Servo_Tas, Motor_Servo_Task, osPriorityNormal, 0, 512);
    Motor_Servo_TasHandle = osThreadCreate(osThread(Motor_Servo_Tas), NULL);

    // Encoder task (PID_Update được gọi ở đây với dt = SAMPLE_TIME_S)
    osThreadDef(Re_Encoder_Task, Encoder_Task, osPriorityNormal, 0, 512);
    Re_Encoder_TaskHandle = osThreadCreate(osThread(Re_Encoder_Task), NULL);
}

