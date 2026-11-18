#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "FreeRTOS.h"
#include "hbl3850_driver.h"
#include "queue.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#ifndef FDCAN_FILTER_MASK
#define FDCAN_FILTER_MASK ((uint32_t)0x00000002U)
#endif

#define OUR_NODE_ID 10U // DroneCAN node ID of this board

#define CMD_PRIORITY 8U
#define COMMAND_TYPE_UNITLESS 0U
#define ARRAY_COMMAND_TYPE 1010U
#define STATUS_COMMAND_TYPE 1011U
#define NODE_STATUS_TYPE 341U
#define TEMPERATURE_STATUS_TYPE 1110U
#define CIRCUIT_STATUS_TYPE 1091U

#define TAIL_START_OF_TRANSFER 0x80U
#define TAIL_END_OF_TRANSFER 0x40U
#define TAIL_TOGGLE 0x20U



/**
 * @brief Reassembly buffer for a multi-frame Actuator.Status transfer. Only the RX callback uses it.
 */
typedef struct {
	uint8_t payload[8];   // the 8 Status payload bytes, filled in across both frames
	uint8_t len;          // bytes received so far (0 means no transfer in progress)
	uint16_t crc;         // CRC from bytes 0–1 of frame 1
	uint8_t transfer_id;  // from the tail byte, must match in frame 2
	bool expect_toggle;   // toggle bit expected in the next frame
} hbl3850_rx_transfer_t;

/**
 * @brief Struct to represent individual motors. Instances would be created for each motor used.
 */
typedef struct {
	uint8_t node_id;
	uint8_t actuator_id;
	QueueHandle_t encoder_queue;
	hbl3850_health_t health;
    bool configured;
	hbl3850_rx_transfer_t status_rx;
	volatile bool heard;
} hbl3850_motor_t;

hbl3850_rx_transfer_t status_rx;

static const uint16_t LOG_WAIT_MS = 0;
static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static uint8_t g_cmd_transfer_id = 0;
static uint32_t can_id = ((uint32_t)CMD_PRIORITY << 24) | ((uint32_t)ARRAY_COMMAND_TYPE << 8) | OUR_NODE_ID;
static hbl3850_motor_t s_motors[HBL3850_MOTOR_COUNT];

w_status_t hbl3850_motor_init(hbl3850_motor_id_t motor, uint8_t node_id, uint8_t actuator_id) {
    if (motor >= HBL3850_MOTOR_COUNT) { return W_INVALID_PARAM; }
    for (hbl3850_motor_id_t m = 0; m < HBL3850_MOTOR_COUNT; ++m) {
        if (!s_motors[m].configured && ((s_motors[m].node_id == node_id) || (s_motors[m].node_id == actuator_id))) {
            return W_INVALID_PARAM;
        }
    }
    if (node_id == 0 || node_id > 127) {
            return W_INVALID_PARAM;
    }
    s_motors[motor].node_id = node_id;
    s_motors[motor].actuator_id = actuator_id;
    s_motors[motor].configured = true;
    return W_SUCCESS;
}

static uint16_t float_to_half(float value)
{
    // TODO: implement
    return 0
}

static float half_to_float(float value)
{
    // TODO: implement
    return 0
}

static void stop_can(void)
{
    HAL_FDCAN_Stop(g_hfdcan);
}

static bool all_motors_heard(void) {
    for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
        if (!s_motors[motor].heard && s_motors[motor].configured)
        {
            return false;
        }
    }
    return true;
}

static int find_motor_by_node_id(uint8_t node_id) {
    for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
        if (s_motors[motor].node_id == node_id && s_motors[motor].configured) {
            return motor;
        }
    }
    return -1;
}

w_status_t hbl3850_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms)
{

    if (NULL == hfdcan)
    {
        log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Invalid pointers");
        return W_INVALID_PARAM;
    }

    g_hfdcan = hfdcan;

    // Create a one-slot encoder queue for each configured motor
    for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
        if (s_motors[motor].configured) {
            s_motors[motor].encoder_queue = xQueueCreate(1, sizeof(hbl3850_encoder_t));
            if (s_motors[motor].encoder_queue == NULL)
            {
                log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Lack of memory to create encoder queue");
                return W_FAILURE;
            }
            s_motors[motor].heard = false;
        }
    }


    // Accept all extended IDs into RX FIFO1
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;

    if (HAL_FDCAN_ConfigFilter(g_hfdcan, &filter) != HAL_OK)
    {
        log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN filter config failed");
        return W_FAILURE;
    }
    
    if (HAL_FDCAN_Start(g_hfdcan) != HAL_OK)
    {
        log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN start failed");
        stop_can();
        return W_FAILURE;
    }


    if (HAL_FDCAN_ActivateNotification(g_hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN activate notification failed");
        stop_can();
        return W_FAILURE;
    }
    // Wait for NodeStatus from every configured motor
    uint32_t start_ms = 0;
    if (timer_get_ms(&start_ms))
    {
        stop_can();
        return W_FAILURE;
    }
    uint32_t now_ms = start_ms;
    while (!(all_motors_heard()) && (now_ms - start_ms <= can_init_timeout_ms))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (timer_get_ms(&now_ms))
        {
            stop_can();
            return W_FAILURE;
        }
    }
    if (!(all_motors_heard()))
    {
        bool all_heard = true;
        for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
        if (!s_motors[motor].heard)
        {
            all_heard = false;
            log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No NodeStatus from node %u", s_motors[motor].node_id);
        }
    }
    if (!all_heard) {
        stop_can();
        return W_FAILURE;
    }
}
    log_text(LOG_WAIT_MS, LOG_LVL_INFO, "hbl3850", "Init successful");
    return W_SUCCESS;
}

static w_status_t hbl3850_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    if ((NULL == g_hfdcan) || (NULL == data) || (len > 8))
    {
        return W_FAILURE;
    }

    // Reinit the CAN module if a bus off state was detected
    FDCAN_ProtocolStatusTypeDef protocolStatus = {};
    HAL_FDCAN_GetProtocolStatus(g_hfdcan, &protocolStatus);
    if (protocolStatus.BusOff)
    {
        CLEAR_BIT(g_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier = ext_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    tx_header.DataLength = len;

    if (HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &tx_header, data) != HAL_OK)
    {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

w_status_t hbl3850_send_position_cmd(hbl3850_motor_id_t motor, float32_t angle_deg)
{
    if (motor >= HBL3850_MOTOR_COUNT || isnan(angle_deg)) { return W_INVALID_PARAM; }
    if (!s_motors[motor].configured) { return W_INVALID_PARAM; }
    uint8_t data[5];
    data[0] = s_motors[motor].actuator_id;
    data[1] = COMMAND_TYPE_UNITLESS;
    float converted_angle = angle_deg / 60;
    if (converted_angle > 1.0)
    {
        converted_angle = 1.0;
    }
    if (converted_angle < -1.0)
    {
        converted_angle = -1.0;
    }
    uint16_t angle_bits = float_to_half(converted_angle);

    data[2] = (uint8_t)(angle_bits & 0xFF);
    data[3] = angle_bits >> 8;
    data[4] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER | g_cmd_transfer_id;
    g_cmd_transfer_id = (g_cmd_transfer_id + 1) & 0x1F;

    return hbl3850_can_transmit_ext(can_id, data, sizeof(data));
}

w_status_t hbl3850_get_encoder(hbl3850_motor_id_t motor, hbl3850_encoder_t *out)
{
    if (motor >= HBL3850_MOTOR_COUNT) { return W_INVALID_PARAM; }
    if (NULL == out)
    {
        return W_FAILURE;
    }

    if (s_motors[motor].encoder_queue == NULL)
    {
        return W_FAILURE;
    }

    if (xQueuePeek(s_motors[motor].encoder_queue, out, 0) == pdPASS)
    {
        return W_SUCCESS;
    }
    return W_FAILURE; // no feedback received yet
}

// Runs from the CAN interrupt every time a frame lands in RX FIFO 1
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if (hfdcan != g_hfdcan)
    {
        return;
    }

    if (0 == (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    // DroneCAN frames always use 29-bit IDs and end with a tail byte
    if ((rx_header.IdType != FDCAN_EXTENDED_ID) || (rx_header.DataLength == 0))
    {
        return;
    }

    // DroneCAN 29-bit ID: type ID = bits 23-8, service = bit 7, source node = bits 6-0
    uint32_t id = rx_header.Identifier;
    uint32_t typeID = (id << 8) >> 16;
    uint32_t service = (id << 24) >> 31;
    uint32_t source = (id << 25) >> 25;
    int motor_index = find_motor_by_node_id(source);

    // Ignore service frames and frames from unknown nodes
    if (service || motor_index == -1)
    {
        return;
    }

    uint32_t now_ms = 0;

    if (timer_get_ms(&now_ms) != W_SUCCESS)
    {
        return;
    }

    if (typeID == STATUS_COMMAND_TYPE)
    {
        // TODO: decode Actuator.Status (1011) into encoder_queue
    }


    if (typeID == NODE_STATUS_TYPE)
    {
        if (rx_header.DataLength < 8) { return; }
        s_motors[motor_index].heard = true;
        uint32_t health = (rx_data[4] >> 6) & 0x03;
        uint32_t mode = (rx_data[4] & 56) >> 3;
        s_motors[motor_index].health.node_health = health;
        s_motors[motor_index].health.node_mode = mode;
        s_motors[motor_index].health.nodestatus_timestamp_ms = now_ms;

    }

    if (typeID == CIRCUIT_STATUS_TYPE)
    {
        if (rx_header.DataLength < 8) { return; }
        float32_t voltage = half_to_float(rx_data[2] | rx_data[3] << 8);
        float32_t current = half_to_float(rx_data[4] | rx_data[5] << 8);
        uint8_t error = rx_data[6];
        s_motors[motor_index].health.voltage_v = voltage;
        s_motors[motor_index].health.current_a = current;
        s_motors[motor_index].health.circuit_err_flags = error;
        s_motors[motor_index].health.circuit_timestamp_ms = now_ms;
    }
    if (typeID == TEMPERATURE_STATUS_TYPE)
    {
        if (rx_header.DataLength < 6) { return; }
        float32_t temperature = half_to_float(rx_data[2] | rx_data[3] << 8) - 273.15f;
        uint8_t error = rx_data[4];
        s_motors[motor_index].health.temperature_c = temperature;
        s_motors[motor_index].health.temp_err_flags = error;
        s_motors[motor_index].health.temp_timestamp_ms = now_ms;
    }
}

