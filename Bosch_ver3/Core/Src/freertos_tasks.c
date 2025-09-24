#include "freertos_tasks.h"

osThreadId CommunicationHandle;
osThreadId Motor_Servo_TasHandle;
osThreadId Re_Encoder_TaskHandle;

// Biến toàn cục dùng chung
extern int motorSpeed;      // PWM output từ PID
extern int steerAngle;      // góc lái từ Pi

PID_Handle_t pid;     // PID controller

/* ================================
   Motor + Servo Task
   ================================ */
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

/* ================================
   FreeRTOS Init
   ================================ */
void MX_FREERTOS_Init(void)
{
    // Init PID (giới hạn output -255..255 cho Motor_SetSpeed)
    PID_Init(&pid, 200.0f, 0.0f, 0.0f, -MOTOR_MAX_INPUT, MOTOR_MAX_INPUT);
    pid.setpoint = 0.3f; // mặc định 0.3 m/s (sẽ được override bởi Pi)

    // Communication task
    osThreadDef(Communication, Communication_Task, osPriorityNormal, 0, 1024);
    CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

    // Motor + Servo task
    osThreadDef(Motor_Servo_Tas, Motor_Servo_Task, osPriorityNormal, 0, 512);
    Motor_Servo_TasHandle = osThreadCreate(osThread(Motor_Servo_Tas), NULL);

    // Encoder task
    osThreadDef(Re_Encoder_Task, Encoder_Task, osPriorityNormal, 0, 512);
    Re_Encoder_TaskHandle = osThreadCreate(osThread(Re_Encoder_Task), NULL);
}
