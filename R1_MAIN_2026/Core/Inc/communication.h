/*
 * communication.h
 *
 *  Created on: Mar 2, 2026
 *      Author: TUF
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

#define CAN_DATA_SIZE    8
#define BOARD_ID_RECEIVE 0x150

void can_transmit(CAN_HandleTypeDef *hcan, uint16_t id,
                  int16_t torque1, int16_t torque2,
                  int16_t torque3, int16_t torque4);

#endif /* INC_COMMUNICATION_H_ */
