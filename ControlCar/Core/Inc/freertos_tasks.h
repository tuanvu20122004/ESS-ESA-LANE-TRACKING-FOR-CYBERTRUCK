#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os.h"

void StartControlTask(void const * argument);
void StartEncoderTask(void const * argument);

#endif /* FREERTOS_TASKS_H */
