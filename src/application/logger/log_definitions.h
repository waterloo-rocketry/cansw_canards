#ifndef LOG__DEFINITIONS_H
#define LOG__DEFINITIONS_H

#include "rocketlib.h"
#include <stdint.h>
// Include headers for structs used in log_data_container_t
#include "application/health_checks/health_checks.h"
#include "application/sensor_handler/sensor_handler.h"
#include "common/gnc/gnc_types.h"
#include "drivers/altimu-10/altimu-10.h"

typedef enum {
#define EVENT(name, id, num_args, message) LOG_EVENTS_##name = id,
#include "log_events.def"
#undef EVENT
} log_event_id_t;

typedef enum {
#define FILE_ID(name, id, file_path) LOG_FILES_##name = id,
#include "log_files.def"
#undef FILE_ID
} log_file_id_t;

#endif
