#ifndef DWT_TIMER_H
#define DWT_TIMER_H
#include "stm32g4xx_hal.h"
static inline void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t DWT_GetUs(uint32_t t_start) {
    return (DWT->CYCCNT - t_start) / 170u;
}
#define DWT_NOW() (DWT->CYCCNT)
#endif
