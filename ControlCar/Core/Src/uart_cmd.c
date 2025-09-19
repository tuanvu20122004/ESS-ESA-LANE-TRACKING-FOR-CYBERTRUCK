#include "uart_cmd.h"
#include "motor.h"
#include "servo.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

uint8_t buffer[30];

void UART_CommandHandler(uint8_t *rx_buffer) {
    if (rx_buffer[0] == 'M' && rx_buffer[1] == '+') {
        uint8_t speed = atoi((char*)(rx_buffer + 2));
        sprintf((char*)buffer, "Motor forward %d/255\r\n", speed);
        Motor_SetSpeed(speed, 1);
        HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 100);

    }
    else if (rx_buffer[0] == 'M' && rx_buffer[1] == '-') {
        uint8_t speed = atoi((char*)(rx_buffer + 2));
        sprintf((char*)buffer, "Motor backward %d/255\r\n", speed);
        Motor_SetSpeed(speed, 0);
        HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 100);
    }
    else if (rx_buffer[0] == 'S') {
        uint8_t angle = atoi((char*)(rx_buffer + 1));
        sprintf((char*)buffer, "Servo %d degrees\r\n", angle);
        Servo_SetAngle(angle);
        HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 100);
    }
    else {
        char err[] = "Error: Invalid Command\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
    }
}
