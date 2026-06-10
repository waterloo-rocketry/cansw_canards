#include "stm32h7xx_hal.h"

// Define here
DEFINE_FAKE_VOID_FUNC(HAL_Init);
DEFINE_FAKE_VOID_FUNC(SCB_InvalidateDCache_by_Addr, void *, int32_t);
DEFINE_FAKE_VOID_FUNC(SCB_CleanDCache_by_Addr, void *, int32_t);