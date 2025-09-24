#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "communication.h"
#include "pid.h"

// Handle các task
extern osThreadId CommunicationHandle;
extern osThreadId Motor_Servo_TasHandle;
extern osThreadId Re_Encoder_TaskHandle;

// Biến toàn cục dùng chung
extern int motorSpeed;      // PWM output từ PID
extern int steerAngle;      // góc lái từ Pi
extern PID_Handle_t pid;    // PID controller

// Khởi tạo FreeRTOS
void MX_FREERTOS_Init(void);

#endif
