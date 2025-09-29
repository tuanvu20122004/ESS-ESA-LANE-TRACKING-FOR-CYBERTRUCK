#include "freertos_tasks.h"

osThreadId CommunicationHandle;
osThreadId Motor_Servo_TasHandle;
osThreadId Re_Encoder_TaskHandle;

int *motorSpeed_t;   // PID output (PWM duty)
int *steerAngle_t;   // Servo angle từ Pi
PID_Handle_t *pid_freetos;    // PID controller

/* ================================
   Motor + Servo Task
   ================================ */

void freetos_task_init(int *motorSpeed, int *steerAngle,PID_Handle_t *pid)
{
	motorSpeed_t = motorSpeed;
	steerAngle_t = steerAngle;
	pid_freetos = pid;

}

static void Motor_Servo_Task(void const * argument)
{
    // Enable cầu H
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);


    for(;;) {
        Motor_SetSpeed(*motorSpeed_t);   // motorSpeed do Encoder_Task + PID tính
        Servo_SetAngle(*steerAngle_t);   // steerAngle do Communication_Task cập nhật
        osDelay(20); // 50Hz update rate
    }
}

/* ================================
   FreeRTOS Init
   ================================ */
void MX_FREERTOS_Init(void)
{
    // Init PID (giới hạn output -255..255 cho Motor_SetSpeed)
    PID_Init(pid_freetos, 150.0f, 50.0f, 10.0f, -MOTOR_MAX_INPUT, MOTOR_MAX_INPUT);
    pid_freetos->setpoint = 0.0f; // mặc định 0.3 m/s (sẽ được override bởi Pi)

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
