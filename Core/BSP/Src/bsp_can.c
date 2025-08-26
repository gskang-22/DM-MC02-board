/**
 * bsp_can.c
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

#include "bsp_can.h"

/**
 * HAL internal callback function that calls abstracted ISR for ease of use.
 * Define can_ISR() elsewhere in code to define behaviour of CAN receive ISR.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan) {
	can_ISR(hcan);
}
void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hcan) {
	can_ISR(hcan);
}

HAL_StatusTypeDef can_get_msg(FDCAN_HandleTypeDef *hcan, FDCAN_RxHeaderTypeDef *rx_msg_header, uint8_t *rx_buffer)
{
    if (hcan->Instance == FDCAN1) {
        return HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, rx_msg_header, rx_buffer);
    } else {
        return HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, rx_msg_header, rx_buffer);
    }
}


uint32_t can_send_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data ){

    FDCAN_TxHeaderTypeDef txHeader;

    // Set up Tx header
    txHeader.Identifier          = id;                 // CAN ID (11-bit if standard, 29-bit if extended)
    txHeader.IdType              = FDCAN_STANDARD_ID;  // or FDCAN_EXTENDED_ID
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;   // Data frame (not remote)
    txHeader.DataLength          = FDCAN_DLC_BYTES_8; // for 8-byte messages
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // Disable BRS unless using CAN FD
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // Classic CAN frame
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;                  // Optional marker

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data) != HAL_OK)
    {
        // Transmission failed
        return 0xFFFFFFFF;  // error code
    }

    return 0;  // success
}

/**
 * @brief  initialises CAN filter, then starts the CAN interrupts
 * for CAN1, messages go into FIFO0. For CAN2, messages go into FIFO1
 *
 * @param *hcan pointer to the CANbus being initialised
 * @param CAN_filterID 32bit CAN ID filterg
 * @param CAN_filtermask 32bit CAN ID mask
 * @usage Call during initialisation to setup filters, start CAN and start ISRs
 */
void can_start(FDCAN_HandleTypeDef *hfdcan, uint32_t filterID, uint32_t filterMask) {
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;   // standard 11-bit ID
    sFilterConfig.FilterIndex  = 0;                   // which filter slot to use
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;   // mask mode
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // send matched frames to FIFO0
    sFilterConfig.FilterID1    = filterID;            // ID to match
    sFilterConfig.FilterID2    = filterMask;          // mask bits

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    // Start the peripheral
    HAL_FDCAN_Start(hfdcan);

    // Enable Rx FIFO0 new message interrupt
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

