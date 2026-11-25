// This file is the "entrypoint" for mocking the STM32H7xx HAL library.
#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H

#include "fff.h"

#ifndef HAL_STATUS_TYPEDEF_DEFINED
#define HAL_STATUS_TYPEDEF_DEFINED
typedef enum {
    HAL_OK = 0x00, // Operation completed successfully
    HAL_ERROR = 0x01, // Error occurred during operation
    HAL_BUSY = 0x02, // Module is busy
    HAL_TIMEOUT = 0x03 // Timeout occurred
} HAL_StatusTypeDef;
typedef enum {
    RESET = 0,
    SET = !RESET
} FlagStatus;
#endif

// Include all other hal mocks, same way the actual hal library works (all src code only includes
// stm32h7xx_hal.h)
#include "hal_adc_mock.h"
#include "hal_fdcan_mock.h"
#include "hal_gpio_mock.h"
#include "hal_i2c_mock.h"
#include "hal_timer_mock.h"
#include "hal_uart_mock.h"
#include "fatfs.h"

// Cortex-M DWT / CoreDebug stubs so host tests can compile DWT_init and cycle probes
typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t CYCCNT;
	volatile uint32_t LAR;
} DWT_Type;
extern DWT_Type DWT_regs;
#define DWT (&DWT_regs)
#define DWT_CTRL_CYCCNTENA_Msk (1u)

typedef struct {
	volatile uint32_t DEMCR;
} CoreDebug_Type;
extern CoreDebug_Type CoreDebug_regs;
#define CoreDebug (&CoreDebug_regs)
#define CoreDebug_DEMCR_TRCENA_Msk (1u)

#ifndef __DSB
#define __DSB() ((void)0)
#endif
#ifndef __ISB
#define __ISB() ((void)0)
#endif

// Declare (but do not define) mock here. Actual definition is in stm32h7xx_hal.c to avoid
// multiple-definitions errors
DECLARE_FAKE_VOID_FUNC(HAL_Init);

#endif // MOCK_STM32H7XX_HAL_H