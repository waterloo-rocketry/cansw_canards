#include "task.h"

DEFINE_FAKE_VALUE_FUNC(TaskHandle_t, xTaskGetCurrentTaskHandle);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xTaskGetSchedulerState);
DEFINE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);
DEFINE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
DEFINE_FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t *, TickType_t);
DEFINE_FAKE_VALUE_FUNC(uint32_t, ulTaskNotifyTakeIndexed, UBaseType_t, BaseType_t, TickType_t);
DEFINE_FAKE_VOID_FUNC(vTaskNotifyGiveIndexedFromISR, TaskHandle_t, UBaseType_t, BaseType_t *);
