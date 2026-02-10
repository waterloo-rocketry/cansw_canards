#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "rocketlib/include/common.h"

/**
 * @brief SD card module health stats
 */
typedef struct {
	bool is_init;
	uint32_t file_create_count;
	uint32_t read_count;
	uint32_t write_count;
	uint32_t err_count;
} sd_card_health_t;

/**
 * @brief Initialize the SD card hardware and create the mutex for thread safety.
 * @pre Must be called after scheduler starts. The HAL sd init inside this uses hal_delay
 * in it, so it will hang forever if the timer interrupt is masked (freertos masks interrupts
 * before scheduler starts).
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t sd_card_init(void);

/**
 * @brief Read from the beginning of a file.
 *
 * @param file_name - The name of the file to read.
 * @param buffer - The buffer to read the file into.
 * @param num_bytes - Number of bytes to read from the file.
 * @param bytes_read - The number of bytes successfully read from the file.
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if file DNE or other fails.
 */
w_status_t sd_card_file_read(const char *file_name, char *buffer, uint32_t num_bytes,
							 uint32_t *bytes_read);

/**
 * @brief Write data to a file on the SD card.
 *
 * Acquires the SD card mutex, opens the file, writes data from buffer,
 * closes the file, then releases the mutex.
 *
 * @param[in]  file_name    Name/path of the file to write to.
 * @param[in]  buffer       Pointer to the data to be written.
 * @param[in]  num_bytes    Number of bytes from buffer to write.
 * @param[in]  append       If true, append to the file. If false, overwrite from the start
 * @param[out] bytes_written Actual number of bytes written (if successful).
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE if file DNE or other fails.
 */
w_status_t sd_card_file_write(const char *file_name, const char *buffer, uint32_t num_bytes,
							  bool append, uint32_t *bytes_written);

/**
 * @brief Create a new file on the SD card.
 *
 * Acquires mutex, creates the file (if it does not exist),
 * closes the file, then releases the mutex.
 *
 * @param[in] file_name  Name/path of the file to create.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE if file already exists or other fails.
 */
w_status_t sd_card_file_create(const char *file_name);

// /**
//  * @brief Delete a file from the SD card.
//  *
//  * Acquires mutex, deletes the file, then releases the mutex.
//  *
//  * @param[in] file_name  Name/path of the file to delete.
//  *
//  * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
//  */
// w_status_t sd_card_file_delete(char *file_name);

/**
 * @brief Check if the SD card is writable.
 *
 * This function verifies that the SD card is in the READY state using HAL_SD_GetCardState.
 * Also checks that the module is initialized (mutex created, etc)
 * @pre MUST be called only after scheduler starts.
 * @param p_sd_handle pointer to the HAL handle for the sd card to check
 * @return w_status_t - W_SUCCESS if the SD card is in READY state, W_FAILURE otherwise.
 */
w_status_t sd_card_is_writable(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief Report SD card module health status
 *
 * Retrieves and reports SD card error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
uint32_t sd_card_get_status(void);

#endif // SD_CARD_H