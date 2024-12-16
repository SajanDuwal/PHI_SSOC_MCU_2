/*
 * obc_interface.c
 *
 *  Created on: Nov 24, 2024
 *      Author: sajanduwal
 */

#include <ssoc2_interface.h>
#include "main.h"
#include "com_debug.h"

uint8_t HANDSHAKE_SUCCESS = 0;

void WAIT_FOR_HANDSHAKE() {
	uint8_t MainCMDHs[5];
	if (HAL_UART_Receive(&hlpuart1, MainCMDHs, 5, 7000) == HAL_OK) {
		myDebug("--> HandShake command received: 0x");
		for (int i = 0; i < 5; i++) {
			myDebug("%x", MainCMDHs[i]);
		}
		myDebug("\n");
		delay_us(1);
		if (MainCMDHs[0] == 0x04 && MainCMDHs[4] == 0xFE) {
			myDebug("--> Command Acknowledged!\n");
			if (HAL_UART_Transmit(&hlpuart1, MainCMDHs, 5, 2000) == HAL_OK) {
				myDebug("--> HandShake ACK sent to MAIN\n");
				HANDSHAKE_SUCCESS = 1;
				delay_us(1);
			}
		} else {
			myDebug("*** Unknown handshake command received\n");
			HANDSHAKE_SUCCESS = 0;
			delay_us(1);
			WAIT_FOR_HANDSHAKE();
		}
	} else {
		delay_us(1);
		HANDSHAKE_SUCCESS = 0;
		WAIT_FOR_HANDSHAKE();
	}
}
