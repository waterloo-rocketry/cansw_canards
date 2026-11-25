#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"
#include "fff.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Task creation functions */
#define xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask)
#define xTaskCreateStatic(pvTaskCode, pcName, ulStackDepth, pvParameters, uxPriority,              \
                          pxStackBuffer, pxTaskBuffer)

/* Task-specific function declarations using FFF */
DECLARE_FAKE_VALUE_FUNC(TaskHandle_t, xTaskGetCurrentTaskHandle);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xTaskGetSchedulerState);
DECLARE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);
DECLARE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
DECLARE_FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t *, TickType_t);
DECLARE_FAKE_VALUE_FUNC(uint32_t, ulTaskNotifyTakeIndexed, UBaseType_t, BaseType_t, TickType_t);
DECLARE_FAKE_VOID_FUNC(vTaskNotifyGiveIndexedFromISR, TaskHandle_t, UBaseType_t, BaseType_t *);

/* Additional task definitions if needed by the application */
#define configMAX_TASK_NAME_LEN 16

#ifdef __cplusplus
}
#endif

#endif /* TASK_H */
