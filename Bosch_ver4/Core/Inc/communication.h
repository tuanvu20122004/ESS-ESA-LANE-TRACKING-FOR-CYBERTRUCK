#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"
#include"pid.h"
// Khai báo hàm
// ================================
// ham init
void communication_init(UART_HandleTypeDef *huart, PID_Handle_t *pid, int *steerAngle);

void Communication_Task(void const * argument);
void Parse_Command(char *rxBuffer);

// UART helper
void UART_Send_IT(const char *msg);// DEBUG

// Feedback gửi về Pi
void Send_SpeedFeedback(float speed, float setpoint, int pwm);//DEBUG

#endif
