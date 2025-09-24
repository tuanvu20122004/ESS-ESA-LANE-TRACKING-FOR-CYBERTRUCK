#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"

// ================================
// Biến global
// ================================


// ================================
// Khai báo hàm
// ================================
void Communication_Task(void const * argument);
void Parse_Command(char *rxBuffer);

// UART helper
void UART_Send_IT(const char *msg);// DEBUG

// Feedback gửi về Pi
void Send_SpeedFeedback(float speed, float setpoint, int pwm);//DEBUG

#endif
