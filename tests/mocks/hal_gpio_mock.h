#ifndef HAL_GPIO_MOCK_H
#define HAL_GPIO_MOCK_H

#include "fff.h"
#include "rocketlib/include/common.h"
#include <stdint.h>

// Copy over HAL GPIO definitions for mocking
#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define GPIOA (GPIO_TypeDef *) (0x40020000UL)
#define GPIOB (GPIO_TypeDef *) (0x40020400UL)
#define GPIOC (GPIO_TypeDef *) (0x40020800UL)
#define GPIOD (GPIO_TypeDef *) (0x40020C00UL)
#define GPIOE (GPIO_TypeDef *) (0x40021000UL)
#define GPIOF (GPIO_TypeDef *) (0x40021400UL)
#define GPIOG (GPIO_TypeDef *) (0x40021800UL)
#define GPIOH (GPIO_TypeDef *) (0x40021C00UL)

typedef void* GPIO_TypeDef; // Mock as a void*

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

// ---------------------
// Declare (but dont define) FFF mocks for HAL_GPIO functions.
// Actual definitions are in the .c file to avoid multiple-definitions errors
// The comments indicate the actual function signatures
// ---------------------

// GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
DECLARE_FAKE_VALUE_FUNC(GPIO_PinState, HAL_GPIO_ReadPin, GPIO_TypeDef*, uint16_t);

// void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
DECLARE_FAKE_VOID_FUNC(HAL_GPIO_WritePin, GPIO_TypeDef*, uint16_t, GPIO_PinState);

// void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
DECLARE_FAKE_VOID_FUNC(HAL_GPIO_TogglePin, GPIO_TypeDef*, uint16_t);

// w_status_t lsm6dsv32x_int1_isr_handler(void);
// w_status_t iis2mdc_handle_drdy_irq(void);
DECLARE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32x_int1_isr_handler);
DECLARE_FAKE_VALUE_FUNC(w_status_t, iis2mdc_handle_drdy_irq);

#endif // HAL_GPIO_MOCK_H
