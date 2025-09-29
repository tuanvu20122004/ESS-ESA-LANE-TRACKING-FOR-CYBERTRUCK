#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "communication.h"
#include "pid.h"

// Biến toàn cục dùng chung


// Khởi tạo FreeRTOS
void MX_FREERTOS_Init(void);

void freetos_task_init(int *motorSpeed, int *steerAngle,PID_Handle_t *pid);

#endif
