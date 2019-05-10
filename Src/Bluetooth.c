

/*
 * Bluetooth.c
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#include "Bluetooth.h"
#include "main.h"
uint8_t bluetooth_memory;
UART_HandleTypeDef* huart_inner;
uint8_t sending_tmp;
uint8_t get_bluetooth(void){
	sending_tmp = bluetooth_memory;
	bluetooth_memory = 0;
	return sending_tmp;
}
void send_bluetooth( uint8_t *word){
	HAL_UART_Transmit_IT(huart_inner,word,1);
}

void start_bluetooth(UART_HandleTypeDef *huart){
      HAL_UART_Init(huart);
	  HAL_UART_Transmit_IT(huart,&bluetooth_memory,1);
	  HAL_Delay(1000);
	  HAL_UART_Receive_IT(huart,&bluetooth_memory,1);
	  huart_inner = huart;
}
uint8_t* interupt_bluetooth_return(void){
	return &bluetooth_memory;
}

