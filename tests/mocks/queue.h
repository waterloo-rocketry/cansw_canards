#ifndef QUEUE_H
#define QUEUE_H

#include "FreeRTOS.h"
#include "fff.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Common Queue types from FreeRTOS */
typedef void *QueueHandle_t;
typedef void *QueueSetHandle_t;
typedef void *QueueSetMemberHandle_t;

/* Declare fake functions for common queue operations */
// QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
DECLARE_FAKE_VALUE_FUNC(QueueHandle_t, xQueueCreate, UBaseType_t, UBaseType_t);
// BaseType_t xQueueSend(QueueHandle_t xQueue, const void * pvItemToQueue, TickType_t xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueSend, QueueHandle_t, const void *, TickType_t);
// BaseType_t xQueueSendToBack(QueueHandle_t xQueue, const void * pvItemToQueue, TickType_t
// xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendToBack, QueueHandle_t, const void *, TickType_t);
// BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *
// pxHigherPriorityTaskWoken);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendFromISR, QueueHandle_t, const void *, BaseType_t *);
// BaseType_t xQueueOverwrite(QueueHandle_t xQueue, const void * pvItemToQueue);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueOverwrite, QueueHandle_t, const void *);
// BaseType_t xQueueReceive(QueueHandle_t xQueue, void * pvBuffer, TickType_t xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceive, QueueHandle_t, void *, TickType_t);
// BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void * pvBuffer, BaseType_t *
// pxHigherPriorityTaskWoken);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceiveFromISR, QueueHandle_t, void *, BaseType_t *);
// BaseType_t xQueuePeek(QueueHandle_t xQueue, void * pvBuffer, TickType_t xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueuePeek, QueueHandle_t, void *, TickType_t);
// UBaseType_t uxQueueMessagesWaiting(QueueHandle_t xQueue);
DECLARE_FAKE_VALUE_FUNC(UBaseType_t, uxQueueMessagesWaiting, QueueHandle_t);
// UBaseType_t uxQueueSpacesAvailable(QueueHandle_t xQueue);
DECLARE_FAKE_VOID_FUNC(vQueueDelete, QueueHandle_t);
// BaseType_t xQueueReset(QueueHandle_t xQueue);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueReset, QueueHandle_t);
// Add overwrite from ISR function
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueOverwriteFromISR, QueueHandle_t, const void *, BaseType_t *);

#ifdef __cplusplus
}
#endif

#endif /* QUEUE_H */