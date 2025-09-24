#include "freertos_tasks.h"



extern int motorSpeed;
extern int steerAngle;

static void Motor_Servo_Task(void const * argument)
{
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);

    for(;;) {
        Motor_SetSpeed(motorSpeed);
        Servo_SetAngle(steerAngle);
        osDelay(200);
    }
}

void MX_FREERTOS_Init(void)
{
    // Communication task
    osThreadDef(Communication, Communication_Task, osPriorityBelowNormal, 0, 1024);
    CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

    // Motor + Servo task
    osThreadDef(Motor_Servo_Tas, Motor_Servo_Task, osPriorityAboveNormal, 0, 512);
    Motor_Servo_TasHandle = osThreadCreate(osThread(Motor_Servo_Tas), NULL);

    // Encoder task
    osThreadDef(Re_Encoder_Task, Encoder_Task, osPriorityNormal, 0, 512);
    Re_Encoder_TaskHandle = osThreadCreate(osThread(Re_Encoder_Task), NULL);
}
