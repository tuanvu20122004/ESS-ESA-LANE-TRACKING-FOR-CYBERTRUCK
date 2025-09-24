#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "communication.h"
#include "pid.h"

//// Handle các task
//extern osThreadId CommunicationHandle;
//extern osThreadId Motor_Servo_TasHandle;
//extern osThreadId Re_Encoder_TaskHandle;



// Khởi tạo FreeRTOS
void MX_FREERTOS_Init(void);

#endif
