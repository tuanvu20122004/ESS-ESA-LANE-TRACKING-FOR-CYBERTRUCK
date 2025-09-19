#include "encoder.h"

static TIM_HandleTypeDef *encoder_htim;
static uint16_t counterInitial = 0;

void Encoder_Init(TIM_HandleTypeDef *htim) {
    encoder_htim = htim;
    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_ALL);
}

uint16_t Encoder_GetDelta(void) {
    uint16_t counterAfter = __HAL_TIM_GET_COUNTER(encoder_htim);
    uint16_t delta;
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder_htim)) {
        delta = (counterAfter >= counterInitial) ?
            (65535 - counterAfter + counterInitial) :
            (counterInitial - counterAfter);
    } else {
        delta = (counterAfter >= counterInitial) ?
            (counterAfter - counterInitial) :
            (65535 - counterInitial + counterAfter + 1);
    }
    counterInitial = counterAfter;
    return delta;
}
