/*
 * obc_interface.h
 *
 *  Created on: Nov 24, 2024
 *      Author: sajanduwal
 */

#ifndef INC_SSOC2_INTERFACE_H_
#define INC_SSOC2_INTERFACE_H_

#include "main.h"

extern UART_HandleTypeDef hlpuart1;

extern uint8_t HANDSHAKE_SUCCESS;

void WAIT_FOR_HANDSHAKE();

#endif /* INC_SSOC2_INTERFACE_H_ */
