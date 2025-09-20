#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

extern volatile int32_t count;
extern float speed_pulse_ms;
extern uint8_t flag;

void Encoder_Task(void const * argument);

#endif
