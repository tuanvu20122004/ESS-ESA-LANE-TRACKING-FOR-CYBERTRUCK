#include "freertos_tasks.h"
#include "uart_cmd.h"
#include "encoder.h"

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Biến dùng chung từ main.c */
extern UART_HandleTypeDef huart2;
extern osSemaphoreId myBinarySem01Handle;
extern uint8_t rx_Buffer[];
extern volatile uint8_t dataComplete;
extern volatile uint8_t rx_index;

/*-------------------- TASK 1: CONTROL TASK ----------------------*/
void StartControlTask(void const * argument)
{
    for(;;)
    {
        /* Chờ semaphore báo có dữ liệu UART */
        osSemaphoreWait(myBinarySem01Handle, osWaitForever);

        if (dataComplete)
        {
            UART_CommandHandler(rx_Buffer);  // Xử lý lệnh
            memset(rx_Buffer, 0, RX_BUFFER_SIZE);
            rx_index = 0;
            dataComplete = 0;

            /* Bật lại UART interrupt để nhận tiếp */
            HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_Buffer[rx_index], 1);
        }
    }
}

/*-------------------- TASK 2: ENCODER TASK ----------------------*/
/*void StartEncoderTask(void const * argument)
{
    char msg[20];
    for(;;)
    {
        uint16_t delta = Encoder_GetDelta();   // Lấy số xung thay đổi
        sprintf(msg, "%d\n", delta);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

        osDelay(1000);  // 1 giây gửi 1 lần
    }
}*/

void StartEncoderTask(void const * argument)
{
    char msg[30];
    static uint16_t fake_counter = 0;

    for(;;)
    {
        // Giả lập encoder: tăng 10 xung mỗi giây
        fake_counter += 10;
        snprintf(msg, sizeof(msg), "Fake encoder: %d\r\n", fake_counter);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

        osDelay(1000);  // Delay 1 giây
    }
}

