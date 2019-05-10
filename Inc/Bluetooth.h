/*
 * Bluetooth.h
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_
#include "main.h"
uint8_t get_bluetooth(void);
void send_bluetooth(uint8_t *word);
void start_bluetooth(UART_HandleTypeDef *huart);
uint8_t* interupt_bluetooth_return(void);

#endif /* BLUETOOTH_H_ */
