/**
 * GPIO module
 * Driver for GPIO pins
 */
#include <stdbool.h>

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "semphr.h"

#include "drivers/gpio/gpio.h"

// Private --------------------------------------------------------------------

/**
 * Private status trackers
 */
typedef struct {
	bool is_init; // Whether module is initialized
	uint32_t accesses; // # of successful r/w
	uint32_t access_fails; // # of failed r/w
	bool err; // Whether module is currently in err state for any reason
} gpio_module_status_t;

static gpio_module_status_t gpio_status = {0};

/**
 * Private gpio pin data for 1 pin
 */
typedef struct {
	GPIO_TypeDef *port; // HAL GPIO port
	uint16_t pin; // HAL GPIO pin bit mask
	SemaphoreHandle_t access_mutex; // access mutex
} gpio_pin_data_t;

/**
 * Map gpio pins enums to their hardware data
 */
static gpio_pin_data_t gpio_map[GPIO_PIN_COUNT] = {
	[GPIO_PIN_RED_LED] = {.port = GPIOE, .pin = GPIO_PIN_9, .access_mutex = NULL},
	[GPIO_PIN_GREEN_LED] = {.port = GPIOE, .pin = GPIO_PIN_10, .access_mutex = NULL},
	[GPIO_PIN_BLUE_LED] = {.port = GPIOE, .pin = GPIO_PIN_11, .access_mutex = NULL},
	[GPIO_PIN_LSM6DSV32X_CS] = {.port = GPIOE, .pin = GPIO_PIN_15, .access_mutex = NULL},
	[GPIO_PIN_LSM6DSV32X_SA0] = {.port = GPIOD, .pin = GPIO_PIN_1, .access_mutex = NULL},
	[GPIO_PIN_ALTIMU_SA0] = {.port = GPIOD, .pin = GPIO_PIN_0, .access_mutex = NULL},
};

// Public ---------------------------------------------------------------------

/**
 * Initialize gpio module. Can be run before scheduler start
 */
w_status_t gpio_init() {
	w_status_t status = W_SUCCESS;

	gpio_status.is_init = false;
	gpio_status.accesses = 0;
	gpio_status.access_fails = 0;
	gpio_status.err = false;

	// Initialize the access mutex for each pin
	for (int i = 0; i < GPIO_PIN_COUNT; i++) {
		gpio_map[i].access_mutex = xSemaphoreCreateMutex();
		if (gpio_map[i].access_mutex == NULL) {
			gpio_status.err = true;
			log_text(10, "gpio", "initfail %d", i);
			status = W_FAILURE;
		}
	}

	if (status == W_SUCCESS) {
		gpio_status.is_init = true;
	} else {
	}

	return status;
}

/**
 * Read the current level of `pin` into `level`.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_read(gpio_pin_t pin, gpio_level_t *level, uint32_t timeout) {
	if (pin >= GPIO_PIN_COUNT || level == NULL) {
		gpio_status.access_fails++;
		return W_INVALID_PARAM;
	}

	if (xSemaphoreTake(gpio_map[pin].access_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
		GPIO_PinState state = HAL_GPIO_ReadPin(gpio_map[pin].port, gpio_map[pin].pin);
		*level = (state == GPIO_PIN_SET) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;

		gpio_status.accesses++;
		xSemaphoreGive(gpio_map[pin].access_mutex);

		return W_SUCCESS;
	} else {
		gpio_status.access_fails++;
		return W_IO_TIMEOUT;
	}
}

/**
 * Set `pin` to `level`. Block for up to `timeout` ms.
 */
w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout) {
	if (pin >= GPIO_PIN_COUNT) {
		gpio_status.access_fails++;
		return W_INVALID_PARAM;
	}

	if (xSemaphoreTake(gpio_map[pin].access_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
		HAL_GPIO_WritePin(gpio_map[pin].port,
						  gpio_map[pin].pin,
						  (level == GPIO_LEVEL_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		gpio_status.accesses++;

		xSemaphoreGive(gpio_map[pin].access_mutex);

		return W_SUCCESS;
	} else {
		gpio_status.access_fails++;
		return W_IO_TIMEOUT;
	}
}

/**
 * Toggle `pin` between high/low from its current level.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_toggle(gpio_pin_t pin, uint32_t timeout) {
	if (pin >= GPIO_PIN_COUNT) {
		gpio_status.access_fails++;
		return W_INVALID_PARAM;
	}

	if (xSemaphoreTake(gpio_map[pin].access_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
		HAL_GPIO_TogglePin(gpio_map[pin].port, gpio_map[pin].pin);
		gpio_status.accesses++;

		xSemaphoreGive(gpio_map[pin].access_mutex);

		return W_SUCCESS;
	} else {
		gpio_status.access_fails++;
		return W_IO_TIMEOUT;
	}
}

/**
 * Reports the current status of the GPIO module
 * @return Status code indicating success or failure
 */
uint32_t gpio_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log operation statistics
	log_text(0,
			 "gpio",
			 "%s Successful accesses: %lu, Failed accesses: %lu",
			 gpio_status.is_init ? "INIT" : "NOT INIT",
			 gpio_status.accesses,
			 gpio_status.access_fails);

	return status_bitfield;
}
