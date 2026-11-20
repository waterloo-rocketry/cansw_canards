#include "littlefs/lfs.h"
#include "octospi.h"
#include "third_party/rocketlib/common/common.h"

#include "application/logger/log.h"
#include "drivers/MX25L25645G/MX25L25645G.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"

// To use 4 lanes to transfer to flash. Since flash supports 4 lanes in both SPI
// and QPI, this does 100% determine OSPI commands (depending on if QSPI is on)
#define SPI_4_ENABLED
#define FLASH_NOTIFICATION_INDEX 1

// TODO: add INIT Checks
static const uint32_t FLASH_SINGLE_DELAY_MS =
	100; // this represents the maximum amount of delay for a single operation
static const uint32_t MAX_RDSTATUS_ATTEMPTS = 2;
static const uint16_t WEL_BIT_MASK = 0x2;
static const uint16_t WIP_BIT_MASK = 0x1;
static const uint16_t P_FAIL_BIT_MASK = 0x20;
static const uint16_t E_FAIL_BIT_MASK = 0x40;

// make sure cache size is always less than 2^16 times of Page size
#define CACHE_SIZE 2048 // Bytes
#define PAGE_SIZE 256 // Bytes
// Each byte of lookahead can track 8 blocks on filesystem (rounding up)
#define LOOKAHEAD_SIZE (((LOG_BUFFER_SIZE - 1) / 8) + 1) // Bytes

static_assert(CACHE_SIZE < (0x10000 * PAGE_SIZE),
              "Flash Chip: Cache size must always be less than 2^16 times of page size");

static const uint16_t SECTOR_SIZE = 4096; // Size of one sector on chip in bytes
static const uint32_t MX25L25645G_FLASH_SIZE = 0x2000000; // Total capacity of chip in bytes

static const uint32_t FLASH_PROG_DELAY_MS = 100;

static const uint32_t FLASH_READ_DUMMY_CYCLE = 8;

static uint8_t lfs_flash_read_buffer[CACHE_SIZE] = {0};
static uint8_t lfs_flash_prog_buffer[CACHE_SIZE] = {0};
static uint8_t lfs_flash_lookahead_buffer[LOOKAHEAD_SIZE] = {0};

/**
 * @brief State of the flash chip
 */
typedef enum {
	FLASH_SPI_NOT_INIT,
	FLASH_SPI_READY,
	FLASH_SPI_COMMANDING, // communicating via HAL_OSPI_Command
	FLASH_SPI_WRITING, // communicating via HAL_OSPI_Transmit
	FLASH_SPI_READING, // communicating via HAL_OSPI_Receive
	FLASH_SPI_ERROR
} flash_spi_state_t;

typedef struct {
	flash_spi_state_t spi_state;
	uint32_t lfs_first_block_offset;
	OSPI_HandleTypeDef *spi_handle;
	gpio_pin_t ncs_pin;
	bool is_init;
} flash_lfs_ctx_t;

static flash_lfs_ctx_t g_flash_ctx = {
	.spi_handle = &hospi1, .spi_state = FLASH_SPI_NOT_INIT, .ncs_pin = GPIO_PIN_FLASH_CS
};

// Used for getting the task to block while waiting for transmissions to complete
static TaskHandle_t task_handler = NULL;

/**
 * @brief Called from callbacks when OSPI errors or aborts
 *
 * Unblocks the task to continue execution in case of error.
 *
 * @param[in] hospi Handle to OSPI
 */
static void handle_HAL_OSPI_error(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	(g_flash_ctx.spi_state) = FLASH_SPI_ERROR;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL != task_handler) {
		vTaskNotifyGiveIndexedFromISR(task_handler, FLASH_NOTIFICATION_INDEX,
		                              &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	task_handler = NULL;
}

/**
 * @brief Called from callbacks when OSPI succeeds
 *
 * Unblocks the task after the end of OSPI transmission.
 *
 * @param[in] hospi Handle to OSPI
 */
static void handle_HAL_OSPI_success(OSPI_HandleTypeDef *hospi, flash_spi_state_t check) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	if (check == (g_flash_ctx.spi_state)) {
		(g_flash_ctx.spi_state) = FLASH_SPI_READY;
	} else {
		(g_flash_ctx.spi_state) = FLASH_SPI_ERROR;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL != task_handler) {
		vTaskNotifyGiveIndexedFromISR(task_handler, FLASH_NOTIFICATION_INDEX,
		                              &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	task_handler = NULL;
}

/**
 * @brief Callback in case OSPI errors
 *
 * Calls handle_HAL_OSPI_error().
 *
 * @param[in] hospi Handle to OSPI
 */
void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi) {
	handle_HAL_OSPI_error(hospi);
}

/**
 * @brief Callback in case OSPI aborts for some reason
 *
 * Calls handle_HAL_OSPI_error().
 *
 * @param[in] hospi Handle to OSPI
 */
void HAL_OSPI_AbortCpltCallback(OSPI_HandleTypeDef *hospi) {
	handle_HAL_OSPI_error(hospi);
}

/**
 * @brief Callback for when OSPI completes a command successfully
 *
 * Calls handle_HAL_OSPI_success().
 *
 * @param[in] hospi Handle to OSPI
 */
void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef *hospi) {
	handle_HAL_OSPI_success(hospi, FLASH_SPI_COMMANDING);
}

/**
 * @brief Callback for when OSPI finishes a read successfully
 *
 * Calls handle_HAL_OSPI_success().
 *
 * @param[in] hospi Handle to OSPI
 */
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi) {
	handle_HAL_OSPI_success(hospi, FLASH_SPI_READING);
}

/**
 * @brief Callback for when OSPI finishes transmitting data successfully
 *
 * Calls handle_HAL_OSPI_success().
 *
 * @param[in] hospi Handle to OSPI
 */
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi) {
	handle_HAL_OSPI_success(hospi, FLASH_SPI_WRITING);
}

/**
 *@brief Send non-blocking command OSPI to flash chip
 *
 * @param[in] cmd OSPI command to send
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if chip is busy before
 * the command is sent. W_IO_ERROR if communication problem.
 */
static w_status_t send_spi_cmd(OSPI_RegularCmdTypeDef *cmd, bool select_ncs) {
	// select chip
	if (select_ncs) {
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_LOW, 0)) {
			return W_IO_ERROR;
		}
	}

	// check to make sure we are SPI bus is not busy
	if (FLASH_SPI_READY != (g_flash_ctx.spi_state)) {
		return W_FAILURE;
	}

	task_handler = xTaskGetCurrentTaskHandle();

	(g_flash_ctx.spi_state) = FLASH_SPI_COMMANDING;
	HAL_StatusTypeDef hal_status = HAL_OSPI_Command_IT((g_flash_ctx.spi_handle), cmd);

	if ((pdTRUE != ulTaskNotifyTakeIndexed(FLASH_NOTIFICATION_INDEX, pdTRUE,
	                                       pdMS_TO_TICKS(FLASH_SINGLE_DELAY_MS))) ||
	    (HAL_OK != hal_status) || (FLASH_SPI_READY != (g_flash_ctx.spi_state))) {
		// unselect on error to not keep CS low
		if (select_ncs) {
			if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
				return W_IO_ERROR;
			}
		}
		return W_IO_ERROR;
	}

	// unselect chip
	if (select_ncs) {
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
			return W_IO_ERROR;
		}
	}

	return W_SUCCESS;
}

/**
 * @brief Read single register from flash chip.
 *
 * @param[in] cmd OSPI command to send
 * @param[in] status Buffer where register data is stored
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if chip is busy before
 * the command is sent. W_IO_ERROR if communication problem.
 */
static w_status_t get_single_reg(OSPI_RegularCmdTypeDef *cmd, uint8_t *status, bool select_ncs) {
	// select chip
	if (select_ncs) {
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_LOW, 0)) {
			return W_IO_ERROR;
		}
	}

	// check to make sure we are SPI bus is not busy
	if (FLASH_SPI_READY != (g_flash_ctx.spi_state)) {
		return W_FAILURE;
	}

	// if needed will add a block
	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), cmd, FLASH_SINGLE_DELAY_MS);

	if (HAL_OK != hal_status) {
		(g_flash_ctx.spi_state) = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	task_handler = xTaskGetCurrentTaskHandle();

	// ready to receive data
	(g_flash_ctx.spi_state) = FLASH_SPI_READING;
	hal_status = HAL_OSPI_Receive_DMA(&hospi1, status);

	if ((pdTRUE != ulTaskNotifyTakeIndexed(FLASH_NOTIFICATION_INDEX, pdTRUE,
	                                       pdMS_TO_TICKS(FLASH_SINGLE_DELAY_MS))) ||
	    (HAL_OK != hal_status) || (FLASH_SPI_READY != (g_flash_ctx.spi_state))) {
		// unselect on error to not keep CS low
		if (select_ncs) {
			if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
				return W_IO_ERROR;
			}
		}
		return W_IO_ERROR;
	}

	// unselect chip
	if (select_ncs) {
		if (W_SUCCESS != gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0)) {
			return W_IO_ERROR;
		}
	}

	return W_SUCCESS;
}

/**
 * @brief Read status register from flash chip.
 *
 * @param[in] status Buffer where status data is stored
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - Inherited from get_single_reg(). W_SUCCESS on success.
 * W_FAILURE if chip is busy before the command is sent. W_IO_ERROR if
 * communication problem.
 */
static w_status_t get_rdsr(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	(cmd_readstatus.Instruction) = 0x05; // Read Status
	(cmd_readstatus.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_readstatus.DataMode) = HAL_OSPI_DATA_1_LINE;
	(cmd_readstatus.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(cmd_readstatus.NbData) = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

/**
 * @brief Read security register from flash chip.
 *
 * @param[in] status Buffer where security register data is stored
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - Inherited from get_single_reg(). W_SUCCESS on success.
 * W_FAILURE if chip is busy before the command is sent. W_IO_ERROR if
 * communication problem.
 */
static w_status_t get_rdscur(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	(cmd_readstatus.Instruction) = 0x2b; // Read Status
	(cmd_readstatus.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_readstatus.DataMode) = HAL_OSPI_DATA_1_LINE;
	(cmd_readstatus.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(cmd_readstatus.NbData) = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

/**
 * @brief Read configuration register from flash chip.
 *
 * @param[in] status Buffer where config data is stored
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - Inherited from get_single_reg(). W_SUCCESS on success.
 * W_FAILURE if chip is busy before the command is sent. W_IO_ERROR if
 * communication problem.
 */
static w_status_t get_rdcr(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	(cmd_readstatus.Instruction) = 0x15; // Read Status
	(cmd_readstatus.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_readstatus.DataMode) = HAL_OSPI_DATA_1_LINE;
	(cmd_readstatus.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(cmd_readstatus.NbData) = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

/**
 * @brief Send write enable command to chip.
 *
 * @param[in] select_ncs If CS should be set to low before sending command
 *
 * @return w_status_t - Inherited from get_single_reg(). W_SUCCESS on success.
 * W_FAILURE if chip is busy before the command is sent. W_IO_ERROR if
 * communication problem.
 */
static w_status_t send_wren(bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_wren = {0};

	(cmd_wren.Instruction) = 0x06; // WREN
	(cmd_wren.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_wren.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(cmd_wren.DataMode) = HAL_OSPI_DATA_NONE;
	(cmd_wren.DummyCycles) = 0;
	(cmd_wren.NbData) = 0;

	return send_spi_cmd(&cmd_wren, select_ncs);
}

/**
 *@brief Read data from chip over OSPI using DMA
 *
 * @param[in] address Address from which to read.
 * @param[out] data Buffer where data is sstored.
 * @param[in] len Size of the data being read.
 *
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if misconfigured
 * registers. W_IO_ERROR if communication problem or timeout on transmit.
 */
static w_status_t flash_read(uint32_t address, uint8_t *data, uint32_t len) {
	// write
	OSPI_RegularCmdTypeDef cmd_read = {0};
#ifdef SPI_4_ENABLED
	(cmd_read.Instruction) = 0xeb; //  Quard read
	(cmd_read.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_read.AddressMode) = HAL_OSPI_ADDRESS_4_LINES;
	(cmd_read.AddressSize) = HAL_OSPI_ADDRESS_32_BITS;
	(cmd_read.Address) = address;
	(cmd_read.DataMode) = HAL_OSPI_DATA_4_LINES;
	(cmd_read.DummyCycles) = FLASH_READ_DUMMY_CYCLE;
	(cmd_read.NbData) = len; // ← bytes
#else
	(cmd_read.Instruction) = 0x03; //  Page Program
	(cmd_read.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_read.AddressMode) = HAL_OSPI_ADDRESS_1_LINE;
	(cmd_read.AddressSize) = HAL_OSPI_ADDRESS_32_BITS;
	(cmd_read.Address) = address;
	(cmd_read.DataMode) = HAL_OSPI_DATA_1_LINE;
	(cmd_read.DummyCycles) = 0;
	(cmd_read.NbData) = len; // ← bytes
#endif

	if (W_SUCCESS != gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0)) {
		return W_IO_ERROR;
	}

	// check to make sure we are SPI bus is not busy
	if (FLASH_SPI_READY != (g_flash_ctx.spi_state)) {
		return W_FAILURE;
	}

	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), &cmd_read, FLASH_SINGLE_DELAY_MS);
	if (HAL_OK != hal_status) {
		(g_flash_ctx.spi_state) = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	task_handler = xTaskGetCurrentTaskHandle();

	(g_flash_ctx.spi_state) = FLASH_SPI_READING;
	hal_status = HAL_OSPI_Receive_DMA((g_flash_ctx.spi_handle), data);

	if ((pdTRUE != ulTaskNotifyTakeIndexed(FLASH_NOTIFICATION_INDEX, pdTRUE,
	                                       pdMS_TO_TICKS(FLASH_SINGLE_DELAY_MS))) ||
	    (HAL_OK != hal_status) || (FLASH_SPI_READY != (g_flash_ctx.spi_state))) {
		// unselect on error to not keep CS low
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
			return W_IO_ERROR;
		}
		return W_IO_ERROR;
	}

	if (W_SUCCESS != gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0)) {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

/**
 *@brief Send data to flash chip over OSPI using DMA
 *
 * @param[in] cmd OSPI command to send. Includes address where to write.
 * @param[in] data Data to send to flash.
 *
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if misconfigured
 * registers. W_IO_ERROR if communication problem or timeout on transmit.
 */
static w_status_t flash_spi_transmit(OSPI_RegularCmdTypeDef *cmd, uint8_t *data) {
	// write
	uint8_t rdsr_response = 0x0;
	for (uint8_t i = 0; i < MAX_RDSTATUS_ATTEMPTS; ++i) {
		if ((W_SUCCESS != send_wren(true)) || (W_SUCCESS != get_rdsr(&rdsr_response, true))) {
			return W_FAILURE;
		}

		if ((WEL_BIT_MASK & rdsr_response) != 0x0) {
			break;
		}
	}

	if ((WEL_BIT_MASK & rdsr_response) == 0x0) { // if still can't stop WIP then that's a WIP error
		return W_FAILURE;
	}

	if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_LOW, 0)) {
		return W_IO_ERROR;
	}
	// check to make sure we are SPI bus is not busy
	if (FLASH_SPI_READY != (g_flash_ctx.spi_state)) {
		return W_FAILURE;
	}

	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), cmd, FLASH_SINGLE_DELAY_MS);
	if (HAL_OK != hal_status) {
		(g_flash_ctx.spi_state) = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	task_handler = xTaskGetCurrentTaskHandle();

	(g_flash_ctx.spi_state) = FLASH_SPI_WRITING;
	hal_status = HAL_OSPI_Transmit_DMA((g_flash_ctx.spi_handle), data);

	if ((pdTRUE != ulTaskNotifyTakeIndexed(FLASH_NOTIFICATION_INDEX, pdTRUE,
	                                       pdMS_TO_TICKS(FLASH_SINGLE_DELAY_MS))) ||
	    (HAL_OK != hal_status) || (FLASH_SPI_READY != (g_flash_ctx.spi_state))) {
		// unselect on error to not keep CS low
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
			return W_IO_ERROR;
		}
		return W_IO_ERROR;
	}

	if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
		return W_IO_ERROR;
	}

	// check WIP is complete
	uint32_t start_time_ms = 0;
	uint32_t curr_time_ms = 0;
	if (W_SUCCESS != timer_get_ms(&start_time_ms) || W_SUCCESS != timer_get_ms(&curr_time_ms)) {
		return W_FAILURE;
	}
	do {
		vTaskDelay(pdMS_TO_TICKS(1)); // needs to wait at least 30 ns between write to read commands
		// (+ extra for writing which should be faster)
		if (W_SUCCESS != get_rdsr(&rdsr_response, true)) {
			return W_IO_ERROR;
		}

		if ((WIP_BIT_MASK & rdsr_response) == 0x00) {
			break;
		}
		timer_get_ms(&curr_time_ms);
	} while (FLASH_PROG_DELAY_MS > (curr_time_ms - start_time_ms));

	if ((WIP_BIT_MASK & rdsr_response) != 0x00) {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

/**
 * @brief Write to single page.
 *
 * @param[in] address Address where to write.
 * @param[in] data Data to write to page.
 * @param[in] len Size of data to write. Cannot be longer than a single page.
 *
 * @return w_status_t - W_SUCCESS if all good. W_FAILURE if misconfigured
 * registers. W_IO_ERROR if error transmitting to chip.
 */
static w_status_t flash_write_page(uint32_t address, uint8_t *data, uint32_t len) {
	// will not write for more than a page
	if (((address % PAGE_SIZE) + len) > PAGE_SIZE) {
		return W_INVALID_PARAM;
	}

	// write
	OSPI_RegularCmdTypeDef cmd_write = {0};
#ifdef SPI_4_ENABLED
	(cmd_write.Instruction) = 0x38; //  Page Program
	(cmd_write.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_write.AddressMode) = HAL_OSPI_ADDRESS_4_LINES;
	(cmd_write.AddressSize) = HAL_OSPI_ADDRESS_32_BITS;
	(cmd_write.Address) = address;
	(cmd_write.DataMode) = HAL_OSPI_DATA_4_LINES;
	(cmd_write.DummyCycles) = 0;
	(cmd_write.NbData) = len; // ← bytes
#else
	(cmd_write.Instruction) = 0x02; //  Page Program
	(cmd_write.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_write.AddressMode) = HAL_OSPI_ADDRESS_1_LINE;
	(cmd_write.AddressSize) = HAL_OSPI_ADDRESS_32_BITS;
	(cmd_write.Address) = address;
	(cmd_write.DataMode) = HAL_OSPI_DATA_1_LINE;
	(cmd_write.DummyCycles) = 0;
	(cmd_write.NbData) = len; // ← bytes
#endif

	if (W_SUCCESS != flash_spi_transmit(&cmd_write, data)) {
		return W_IO_ERROR;
	}

	// check if wrote sucessfully
	uint8_t rdscur_response = 0;
	if (W_SUCCESS != get_rdscur(&rdscur_response, true)) {
		return W_IO_ERROR;
	} else if ((P_FAIL_BIT_MASK & rdscur_response) != 0x0) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Erase sector on flash.
 *
 * @param[in] address Address of sector to erase.
 *
 * @return w_status_t - W_SUCCESS if all good. W_FAILURE if misconfigured
 * registers. W_IO_ERROR if error transmitting to chip.
 */
static w_status_t flash_erase_sector(uint32_t address) {
	// erase
	uint8_t rdsr_response = 0x0;
	for (uint8_t i = 0; i < MAX_RDSTATUS_ATTEMPTS; ++i) {
		if ((W_SUCCESS != send_wren(true)) || (W_SUCCESS != get_rdsr(&rdsr_response, true))) {
			return W_FAILURE;
		}

		if ((WEL_BIT_MASK & rdsr_response) != 0x0) {
			break;
		}
	}

	if ((WEL_BIT_MASK & rdsr_response) == 0x0) { // if still can't stop WIP then that's a WIP error
		return W_FAILURE;
	}

	// write
	OSPI_RegularCmdTypeDef cmd_erase = {0};
	(cmd_erase.Instruction) = 0x20;
	(cmd_erase.AddressSize) = HAL_OSPI_ADDRESS_32_BITS;
	(cmd_erase.DataMode) = HAL_OSPI_DATA_NONE;
	(cmd_erase.DummyCycles) = 0;
	(cmd_erase.NbData) = 0;
	(cmd_erase.Address) = address;
#if (defined(SPI_4_ENABLED) && defined(QPI_ENABLED))
	(cmd_erase.InstructionMode) = HAL_OSPI_INSTRUCTION_4_LINES;
	(cmd_erase.AddressMode) = HAL_OSPI_ADDRESS_4_LINES;
#else
	(cmd_erase.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(cmd_erase.AddressMode) = HAL_OSPI_ADDRESS_1_LINE;
#endif

	if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_LOW, 0)) {
		return W_IO_ERROR;
	}
	// check to make sure we are SPI bus is not busy
	if (FLASH_SPI_READY != (g_flash_ctx.spi_state)) {
		return W_FAILURE;
	}

	task_handler = xTaskGetCurrentTaskHandle();

	(g_flash_ctx.spi_state) = FLASH_SPI_COMMANDING;
	HAL_StatusTypeDef hal_status = HAL_OSPI_Command_IT((g_flash_ctx.spi_handle), &cmd_erase);

	if ((pdTRUE != ulTaskNotifyTakeIndexed(FLASH_NOTIFICATION_INDEX, pdTRUE,
	                                       pdMS_TO_TICKS(FLASH_SINGLE_DELAY_MS))) ||
	    (HAL_OK != hal_status) || (FLASH_SPI_READY != (g_flash_ctx.spi_state))) {
		// unselect on error to not keep CS low
		if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
			return W_IO_ERROR;
		}
		return W_IO_ERROR;
	}

	if (W_SUCCESS != gpio_write((g_flash_ctx.ncs_pin), GPIO_LEVEL_HIGH, 0)) {
		return W_IO_ERROR;
	}

	// check WIP is complete
	uint32_t start_time_ms = 0;
	uint32_t curr_time_ms = 0;
	if (W_SUCCESS != timer_get_ms(&start_time_ms) || W_SUCCESS != timer_get_ms(&curr_time_ms)) {
		return W_FAILURE;
	}
	do {
		vTaskDelay(pdMS_TO_TICKS(1)); // needs to wait at least 30 ns between write to read commands
		                              // (+ extra for writing which should be faster)
		if (W_SUCCESS != get_rdsr(&rdsr_response, true)) {
			return W_IO_ERROR;
		}

		if ((WIP_BIT_MASK & rdsr_response) == 0x00) {
			break;
		}
		timer_get_ms(&curr_time_ms);
	} while (FLASH_PROG_DELAY_MS > (curr_time_ms - start_time_ms));
	if ((WIP_BIT_MASK & rdsr_response) != 0x00) {
		return W_IO_ERROR;
	}

	// check if wrote successfully
	uint8_t rdscur_response = 0;
	if (W_SUCCESS != get_rdscur(&rdscur_response, true)) {
		return W_IO_ERROR;
	} else if ((E_FAIL_BIT_MASK & rdscur_response) != 0x0) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Read a region in a block
 *
 * Used by littlefs. Negative error codes are propagated to the user
 * (see littlefs for meaning of codes).
 *
 * @param[in] c littlefs config. Needed by littlefs.
 * @param[in] block Address of the block to be read.
 * @param[in] off Offset from the block address.
 * @param[out] buffer Buffer to store read data.
 * @param[in] size Amount of data to read from block.
 *
 * @return int - 1 if chip is not init. See littlefs docs for other codes.
 */
static int MX25L25645G_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                                void *buffer, lfs_size_t size) {
	// make sure is init
	if (!(g_flash_ctx.is_init)) {
		return W_FAILURE;
	}

	if ((size % (c->read_size)) != 0) {
		return LFS_ERR_INVAL;
	}

	uint32_t start_address = (g_flash_ctx.lfs_first_block_offset) + (block * (c->block_size)) + off;

	// no limit to
	if (W_SUCCESS != flash_read(start_address, buffer, size)) {
		return LFS_ERR_IO;
	}
	return LFS_ERR_OK;
}

/**
 * @brief Program a region in a block
 *
 * Used by littlefs. The block must have previously been erased.
 *
 * @param[in] c littlefs config. Needed by littlefs.
 * @param[in] block Address of the block to be read.
 * @param[in] off Offset from the block address.
 * @param[out] buffer Buffer for data to write.
 * @param[in] size Amount of data to write to block.
 *
 * @return int - 1 if chip is not init. See littlefs docs for other codes.
 */
static int MX25L25645G_lfs_prg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                               const void *buffer, lfs_size_t size) {
	// make sure is init
	if (!(g_flash_ctx.is_init)) {
		return W_FAILURE;
	}

	uint32_t curr_address = (g_flash_ctx.lfs_first_block_offset) + (block * (c->block_size)) + off;

	uint32_t curr_write_size = size;
	uint8_t *curr_buffer = (uint8_t *)buffer;

	// overflows page
	for (uint16_t i = 0; i < ((c->cache_size) / (c->prog_size)); ++i) {
		if ((size + (curr_address % (c->prog_size))) > (c->prog_size)) {
			curr_write_size = (c->prog_size) - (curr_address % (c->prog_size));
		} else {
			curr_write_size = size;
		}

		// write to end of page first
		if (W_SUCCESS != flash_write_page(curr_address, curr_buffer, curr_write_size)) {
			return LFS_ERR_IO;
		}

		// increment for next write
		curr_address += curr_write_size;
		curr_buffer += curr_write_size;

		// calculate the
		if (size > curr_write_size) {
			size -= curr_write_size;
		} else { // if we have written everything at this point
			break;
		}
	}

	if (size > curr_write_size) {
		return LFS_ERR_IO;
	} else {
		return LFS_ERR_OK;
	}
}

/**
 * @brief Erase a block.
 *
 * Used by littlefs. A block must be erased before being programmed.
 * The state of an erased block is undefined. See littlefs for meaning of codes.
 *
 * @param[in] c Passes littlefs config. Needed by littlefs.
 * @param[in] block Block address to erase.
 */
static int MX25L25645G_lfs_erase(const struct lfs_config *c, lfs_block_t block) {
	// make sure is init
	if (!(g_flash_ctx.is_init)) {
		return W_FAILURE;
	}

	if (((c->block_size) % SECTOR_SIZE) != 0) {
		return LFS_ERR_INVAL;
	}

	// TODO: future implement block erase if we can
	if (W_SUCCESS !=
	    flash_erase_sector((g_flash_ctx.lfs_first_block_offset) + (block * (c->block_size)))) {
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/**
 * @brief Sync files to chip if not written yet
 *
 * Used by littlefs. Sync the state of the underlying block device.
 *
 * @param[in] c Needed by littlefs
 */
static int MX25L25645G_lfs_sync(const struct lfs_config *c) {
	return LFS_ERR_OK;
}

// TODO: set static buffers!!!!!!!!!!!!!
// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = MX25L25645G_lfs_read,
	.prog = MX25L25645G_lfs_prg,
	.erase = MX25L25645G_lfs_erase,
	.sync = MX25L25645G_lfs_sync,

	// block device configuration
	.read_size = 1,
	.prog_size = PAGE_SIZE,
	.block_size = SECTOR_SIZE,
	.block_count = (MX25L25645G_FLASH_SIZE / SECTOR_SIZE),
	.block_cycles = 500,
	.cache_size = CACHE_SIZE,
	.lookahead_size = LOOKAHEAD_SIZE,
	.compact_thresh = -1,

	.read_buffer = lfs_flash_read_buffer,
	.prog_buffer = lfs_flash_prog_buffer,
	.lookahead_buffer = lfs_flash_lookahead_buffer,

	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1
};

w_status_t mx25l25645g_init(void) {
	if (g_flash_ctx.is_init) {
		return W_FAILURE;
	}

	(g_flash_ctx.spi_state) = FLASH_SPI_READY;

	// SET UP THE CHIP
	// enable QE and set dc 0 and dc 1 to 10 for Quard IO Fast Read
	OSPI_RegularCmdTypeDef wrsr_cmd = {0};

	// Enable 4 Byte address
	(wrsr_cmd.Instruction) = 0x01; // WRSR
	(wrsr_cmd.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(wrsr_cmd.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(wrsr_cmd.DataMode) = HAL_OSPI_DATA_1_LINE;
	(wrsr_cmd.DummyCycles) = 0;
	(wrsr_cmd.NbData) = 2;

	uint8_t update_reg[2] = {
		0x40, // Status Reg set QE to enabled
		0xa0 // CONFIG Reg set DC[1:0] to 10 and 4Byte on
	};

	// write the setting
	if (W_SUCCESS != flash_spi_transmit(&wrsr_cmd, update_reg)) {
		return W_IO_ERROR;
	}

	OSPI_RegularCmdTypeDef en4b_cmd = {0};

	// Enable 4 Byte address
	(en4b_cmd.Instruction) = 0xb7; // EN4B
	(en4b_cmd.InstructionMode) = HAL_OSPI_INSTRUCTION_1_LINE;
	(en4b_cmd.AddressMode) = HAL_OSPI_ADDRESS_NONE;
	(en4b_cmd.DataMode) = HAL_OSPI_DATA_NONE;
	(en4b_cmd.DummyCycles) = 0;
	(en4b_cmd.NbData) = 0;

	if (W_SUCCESS != send_spi_cmd(&en4b_cmd, true)) {
		return W_IO_ERROR;
	}

	// check that our registers have been set correctly
	// check rdsr
	uint8_t rdsr_response = 0;
	if (W_SUCCESS != get_rdsr(&rdsr_response, true)) {
		return W_IO_ERROR;
	}
	if ((0xfc & rdsr_response) != update_reg[0]) { // checks all fields except WEL adn WIP
		return W_FAILURE;
	}

	// check rdcr
	uint8_t rdcr_response = 0;
	if (W_SUCCESS != get_rdcr(&rdcr_response, true)) {
		return W_IO_ERROR;
	}
	if ((0xf8 & rdcr_response) != update_reg[1]) { // checks all fields except ODS and reserved
		return W_FAILURE;
	}

	(g_flash_ctx.is_init) = true;

	return W_SUCCESS;
}

w_status_t mx25l25645g_mount(lfs_t *p_lfs, const uint32_t first_block_offset) {
	// make sure is init
	if (!(g_flash_ctx.is_init)) {
		return W_FAILURE;
	}

	if ((first_block_offset % (cfg.block_size)) != 0) {
		return W_INVALID_PARAM; // block offset should be a multiple of block size
	}
	memset(p_lfs, 0, sizeof(lfs_t));

	(g_flash_ctx.lfs_first_block_offset) = first_block_offset;

	int res = lfs_mount(p_lfs, &cfg);
	if (0 != res) {
		// try to reformat
		res = lfs_format(p_lfs, &cfg);
		if (0 != res) {
			return W_FAILURE;
		}
		res = lfs_mount(p_lfs, &cfg);
		if (0 != res) {
			return W_FAILURE;
		}
	}

	return W_SUCCESS;
}
