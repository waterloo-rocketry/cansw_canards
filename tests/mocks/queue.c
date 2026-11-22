#include "queue.h"

// Define fake functions for queue operations
DEFINE_FAKE_VALUE_FUNC(QueueHandle_t, xQueueCreate, UBaseType_t, UBaseType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueSend, QueueHandle_t, const void *, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendToBack, QueueHandle_t, const void *, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendFromISR, QueueHandle_t, const void *, BaseType_t *);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueOverwrite, QueueHandle_t, const void *);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceive, QueueHandle_t, void *, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceiveFromISR, QueueHandle_t, void *, BaseType_t *);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueuePeek, QueueHandle_t, void *, TickType_t);
DEFINE_FAKE_VALUE_FUNC(UBaseType_t, uxQueueMessagesWaiting, QueueHandle_t);
DEFINE_FAKE_VOID_FUNC(vQueueDelete, QueueHandle_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueReset, QueueHandle_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueOverwriteFromISR, QueueHandle_t, const void *, BaseType_t *);
