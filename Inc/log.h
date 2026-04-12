#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"

#include "canlib.h"

#define PAGE_SIZE 4096
#define NUM_BUFFERS 4
#define SIGNATURE_SIZE 4
#define MAX_FILE_SIZE_PAGES 262144

w_status_t temp_log_init(void);
void log_handle_incoming_message(const can_msg_t *msg, uint32_t timestamp);
void log_heartbeat(void);

#endif