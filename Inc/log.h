#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"

#include "canlib.h"

#define PAGE_SIZE 4096
#define NUM_BUFFERS 4
#define SIGNATURE_SIZE 4
#define MAX_FILE_SIZE_PAGES 262144

w_status_t lfslog_init(void);
void lfslog_handle_incoming_message(const can_msg_t *msg, uint32_t timestamp);
void lfslog_heartbeat(void);

#endif