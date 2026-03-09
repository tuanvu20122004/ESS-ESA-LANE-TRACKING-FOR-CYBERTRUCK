#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"
#include "pid.h"

// Khai báo hàm
// ================================
void Communication_Task(void const * argument);
void Parse_Command(char *rxBuffer);

// UART helper
void UART_Send_IT(const char *msg);// Ham nay cung de DEBUG

// Feedback gửi về Pi
void Send_SpeedFeedback(float setpoint, float speed);
void Send_PWM_Feedback(uint16_t pwm, float speed);// Ham nay de DEBUG

#endif
