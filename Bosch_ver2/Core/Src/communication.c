#include "communication.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define RX_BUF_SIZE 64

uint8_t rxByte;
char rxBuffer[RX_BUF_SIZE];
uint16_t rxIndex = 0;

int motorSpeed = 0;
int steerAngle = 0;

extern volatile int32_t count;
extern float speed_pulse_ms;
extern uint8_t flag;

char msg[64];

// ✅ Hàm gửi encoder feedback
void Send_EncoderData(int32_t count, float speed)
{
    snprintf(msg, sizeof(msg), "ENC,%ld,%.2f\r\n", count, speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}


// ✅ Hàm parse lệnh CMD
void Parse_Command(char *rxBuffer)
{
    if (strncmp(rxBuffer, "CMD", 3) == 0)
    {
        int spd, ang;
        // Dùng sscanf để parse: "CMD,<speed>,<angle>"
        if (sscanf(rxBuffer, "CMD,%d,%d", &spd, &ang) == 2)
        {
            motorSpeed = spd;
            steerAngle = ang;

            // ✅ Trả ACK khi nhận lệnh hợp lệ
            char ackMsg[64];
            snprintf(ackMsg, sizeof(ackMsg), "ACK,%d,%d\r\n", motorSpeed, steerAngle);
            HAL_UART_Transmit(&huart2, (uint8_t*)ackMsg, strlen(ackMsg), 100);
        }
        else
        {
            // ❌ Sai format → trả về ERR
            char errMsg[] = "ERR,BadFormat\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), 100);
        }
    }
}


// ✅ Callback UART nhận dữ liệu
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // dùng UART2 cho cả Tx/Rx
    {
        if (rxByte == '\n') {
            rxBuffer[rxIndex] = '\0';
            Parse_Command(rxBuffer); // gọi parser
            rxIndex = 0;
            memset(rxBuffer, 0, RX_BUF_SIZE);
        } else {
            if (rxIndex < RX_BUF_SIZE - 1) {
                rxBuffer[rxIndex++] = rxByte;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}

// ✅ Task giao tiếp
void Communication_Task(void const * argument)
{
    for(;;)
    {
        if (flag == 1)
        {
            flag = 0;
            Send_EncoderData(count, speed_pulse_ms);
        }
        osDelay(500);
    }
}
