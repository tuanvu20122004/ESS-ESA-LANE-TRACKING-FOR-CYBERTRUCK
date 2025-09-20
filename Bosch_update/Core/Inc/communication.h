#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"

extern int motorSpeed;
extern int steerAngle;

void Communication_Task(void const * argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// ✅ Hàm mới
void Send_EncoderData(int32_t count, float speed);
void Parse_Command(char *rxBuffer);

#endif
