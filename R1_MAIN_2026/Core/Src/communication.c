/*
 * communication.c
 *
 *  Created on: Mar 2, 2026
 *      Author: TUF
 */

#include "main.h"
#include "communication.h"

void can_transmit(CAN_HandleTypeDef *hcan, uint16_t id,
                  int16_t msg1, int16_t msg2, int16_t msg3, int16_t msg4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
    uint32_t pTxMailbox;

    if (hcan == NULL) {
        return;
    }

    if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
        return;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        return;
    }

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = CAN_DATA_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;

    data[0] = msg1 >> 8;
    data[1] = msg1;
    data[2] = msg2 >> 8;
    data[3] = msg2;
    data[4] = msg3 >> 8;
    data[5] = msg3;
    data[6] = msg4 >> 8;
    data[7] = msg4;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &pTxMailbox) != HAL_OK) {
        return;
    }
}
