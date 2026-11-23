/**
 * GPIO module
 * Driver for GPIO pins
 */
#include <stdbool.h>

#include "main.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "semphr.h"

#include "drivers/gpio/gpio.h"
#include "drivers/iis2mdc/IIS2MDC.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"

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
	[GPIO_PIN_RED_LED] = {.port = LED_R_GPIO_Port, .pin = LED_R_Pin, .access_mutex = NULL},
	[GPIO_PIN_GREEN_LED] = {.port = LED_G_GPIO_Port, .pin = LED_G_Pin, .access_mutex = NULL},
	[GPIO_PIN_BLUE_LED] = {.port = LED_B_GPIO_Port, .pin = LED_B_Pin, .access_mutex = NULL},
	[GPIO_PIN_FLASH_CS] = {.port = FLASH_CS_GPIO_Port, .pin = FLASH_CS_Pin, .access_mutex = NULL},

	[GPIO_PIN_EN_EXT_5V] =
		{.port = EN_EXT_5V_GPIO_Port, .pin = EN_EXT_5V_Pin, .access_mutex = NULL},
	[GPIO_PIN_PWR_EN] = {.port = PWR_EN_GPIO_Port, .pin = PWR_EN_Pin, .access_mutex = NULL},
	[GPIO_PIN_ADXRS649_ST1] =
		{.port = ADXRS_ST1_GPIO_Port, .pin = ADXRS_ST1_Pin, .access_mutex = NULL},
	[GPIO_PIN_ADXRS649_ST2] =
		{.port = ADXRS_ST2_GPIO_Port, .pin = ADXRS_ST2_Pin, .access_mutex = NULL},
	[GPIO_PIN_ADS1219_INT] = {.port = ADC_INT_GPIO_Port, .pin = ADC_INT_Pin, .access_mutex = NULL},
	[GPIO_PIN_ADXL380_INT0] =
		{.port = ADXL_INT_GPIO_Port, .pin = ADXL_INT_Pin, .access_mutex = NULL},
	[GPIO_PIN_PG_EXT_5V] =
		{.port = PG_EXT_5V_GPIO_Port, .pin = PG_EXT_5V_Pin, .access_mutex = NULL},
	[GPIO_PIN_BAT_FLT1] = {.port = BAT_FLT1_GPIO_Port, .pin = BAT_FLT1_Pin, .access_mutex = NULL},
	[GPIO_PIN_BAT_FLT2] = {.port = BAT_FLT2_GPIO_Port, .pin = BAT_FLT2_Pin, .access_mutex = NULL},
	[GPIO_PIN_CHG_MUX_EN] = {
		.port = CHG_MUX_EN_GPIO_Port, .pin = CHG_MUX_EN_Pin, .access_mutex = NULL
	},
};

/**
 * Struct representing GPIO pin interrupt set
 */
typedef struct {
	gpio_isr_callback_t callback;
	uint16_t gpio_pin;
} gpio_isr_callback_data_t;

static gpio_isr_callback_data_t g_isr_map[MAX_GPIO_INTERRUPTS] = {0};
static uint8_t g_isr_max_index = 0;

// Public ---------------------------------------------------------------------

/**
 * Initialize gpio module. Can be run before scheduler start
 */
w_status_t gpio_init(void) {
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
			log_text(10, LOG_LVL_FATAL, "gpio", "initfail %d", i);
			status = W_FAILURE;
		}
	}

	if (W_SUCCESS == status) {
		gpio_status.is_init = true;
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
		HAL_GPIO_WritePin(gpio_map[pin].port, gpio_map[pin].pin,
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
health_status_t gpio_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log operation statistics
	log_text(0, LOG_LVL_INFO, "gpio", "%s Successful accesses: %lu, Failed accesses: %lu",
	         gpio_status.is_init ? "INIT" : "NOT INIT", gpio_status.accesses,
	         gpio_status.access_fails);

	health_status_t status = {
		.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
		.module_id = CANARDS_MODULE_ID_GPIO,
		.error_bitfield = 0
	};

	return status;
}

/**
 * @brief Register a gpio pin with an interrupt callback function
 * @param[in] p_callback_fcn is the pointer to callback function
 * @param[in] gpio_pin is the pin number for the associated GPIO pin
 * @return the status of the registration
 */
w_status_t gpio_register_interrupt_callback(const gpio_isr_callback_t p_callback_fcn,
                                            const uint16_t gpio_pin) {
	if (NULL == p_callback_fcn) {
		log_text(10, LOG_LVL_WARN, "gpio", "Failed to register gpio callback on pin %d", gpio_pin);
		return W_INVALID_PARAM;
	}

	if (g_isr_max_index >= MAX_GPIO_INTERRUPTS) {
		log_text(10, LOG_LVL_WARN, "gpio", "Maxed out gpio isr interrupts with pin %d", gpio_pin);
		return W_FAILURE;
	}

	// make sure each unique GPIO pin only has a single callback
	for (uint8_t isr_index = 0; isr_index < g_isr_max_index; ++isr_index) {
		if (g_isr_map[isr_index].gpio_pin == gpio_pin) {
			log_text(10, LOG_LVL_WARN, "gpio", "overlap registration of gpio callback on pin %d",
			         gpio_pin);
			return W_INVALID_PARAM;
		}
	}

	// create callback
	g_isr_map[g_isr_max_index].callback = p_callback_fcn;
	g_isr_map[g_isr_max_index].gpio_pin = gpio_pin;

	// increment counter
	g_isr_max_index++;

	return W_SUCCESS;
}

/**
 * @brief HAL interrupt callback for GPIO pins.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// search through and call the respected callback
	for (uint8_t isr_index = 0; isr_index < g_isr_max_index; isr_index++) {
		if (g_isr_map[isr_index].gpio_pin == GPIO_Pin) {
			g_isr_map[isr_index].callback();
			return;
		}
	}
}
