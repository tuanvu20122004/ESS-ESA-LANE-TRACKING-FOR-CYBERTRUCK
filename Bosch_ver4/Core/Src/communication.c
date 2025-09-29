#include "communication.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pid.h"

// UART handle
//extern UART_HandleTypeDef huart2;
//
//// PID handle được khai báo ở freertos_tasks.c
//extern PID_Handle_t pid;
//extern int steerAngle;   // góc lái từ Pi
UART_HandleTypeDef *huart2_communication;
PID_Handle_t *pid_communication;
int *steerAngle_communication;


void communication_init(UART_HandleTypeDef *huart, PID_Handle_t *pid, int *steerAngle)
{
	huart2_communication = huart;
	pid_communication = pid;
	steerAngle_communication = steerAngle;
}


uint8_t rxByte;

/* ================================
   Ring Buffer RX
   ================================ */
#define RX_BUF_SIZE     64
#define RING_BUF_SIZE   128
#define TX_QUEUE_SIZE   4

typedef struct {
    uint8_t buffer[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

static RingBuffer rxFifo;

/* ================================
   TX queue cho UART (non-blocking)
   ================================ */
static char txBuf[TX_QUEUE_SIZE][64];
static volatile uint8_t txHead = 0, txTail = 0;
static volatile uint8_t txBusy = 0;

/* ================================
   RingBuffer helper
   ================================ */
static inline void RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    uint16_t next = (rb->head + 1) % RING_BUF_SIZE;
    if (next != rb->tail) { // tránh tràn
        rb->buffer[rb->head] = data;
        rb->head = next;
    }
}

static inline int RingBuffer_Get(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) return 0; // rỗng
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
    return 1;
}

/* ================================
   UART TX (non-blocking)
   ================================ */
void UART_Send_IT(const char *msg)
{
    uint8_t next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) return; // queue full → bỏ

    strncpy(txBuf[txHead], msg, sizeof(txBuf[0]));
    txHead = next;

    if (!txBusy) {
        txBusy = 1;
        HAL_UART_Transmit_IT(huart2_communication, (uint8_t*)txBuf[txTail], strlen(txBuf[txTail]));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        txTail = (txTail + 1) % TX_QUEUE_SIZE;
        if (txTail != txHead) {
            HAL_UART_Transmit_IT(huart2_communication, (uint8_t*)txBuf[txTail], strlen(txBuf[txTail]));
        } else {
            txBusy = 0;
        }
    }
}

/* ================================
   UART RX Callback
   ================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        RingBuffer_Put(&rxFifo, rxByte);
        HAL_UART_Receive_IT(huart2_communication, &rxByte, 1); // nhận tiếp
    }
}

/* ================================
   Command Parser
   ================================ */
void Parse_Command(char *rxBuffer)
{
    if (strncmp(rxBuffer, "CMD", 3) == 0) {
        float spd;  // vận tốc m/s
        int ang;    // góc lái
        if (sscanf(rxBuffer, "CMD,%f,%d", &spd, &ang) == 2) {
            // Gán setpoint trực tiếp (m/s)
            pid_communication->setpoint = spd;

            // Cập nhật góc lái
            *steerAngle_communication = ang;
        }
    }
}

/* ================================
   Feedback gửi về Pi
   ================================ */
void Send_SpeedFeedback(float setpoint, float speed, int pwm)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "FB,SET,%.3f,SPD,%.3f,PWM,%d\r\n",
             setpoint, speed, pwm);
    UART_Send_IT(msg);
}

/* ================================
   Communication Task
   ================================ */
void Communication_Task(void const * argument)
{
    static char cmdBuf[RX_BUF_SIZE];
    static uint16_t cmdIndex = 0;
    uint8_t c;

    // init FIFO
    rxFifo.head = rxFifo.tail = 0;
    HAL_UART_Receive_IT(huart2_communication, &rxByte, 1);

    for(;;)
    {
        // đọc FIFO và parse command
        while (RingBuffer_Get(&rxFifo, &c)) {
            if (c == '\n') {
                cmdBuf[cmdIndex] = '\0';
                Parse_Command(cmdBuf);
                cmdIndex = 0;
            } else {
                if (cmdIndex < sizeof(cmdBuf)-1) {
                    cmdBuf[cmdIndex++] = c;
                }
            }
        }

        osDelay(1); // tránh chiếm CPU
    }
}
