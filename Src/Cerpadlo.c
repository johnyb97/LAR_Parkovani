/*
 * Cerpadlo.c
 *
 *  Created on: Apr 29, 2019
 *      Author: janbe
 */

#ifndef CERPADLO_C_
#define CERPADLO_C_
#include "Cerpadlo.h"
#include "motorcontrol.h"
#include "main.h"
int counter_cerpadlo;
int cerpadlo1;
int cerpadlo2;
void HAL_IncTick(void)
{
  uwTick++;
  counter_cerpadlo++;
  if (counter_cerpadlo>100){
	  counter_cerpadlo = 0;
  }
  if (counter_cerpadlo<(100-max_rychlost_cerpadla)){
	  HAL_GPIO_WritePin(Cerpadlo_posun1_GPIO_Port,Cerpadlo_posun1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Cerpadlo_posun2_GPIO_Port,Cerpadlo_posun2_Pin,GPIO_PIN_RESET);
  }
  if (counter_cerpadlo>(100-max_rychlost_cerpadla)){
	  HAL_GPIO_WritePin(Cerpadlo_posun1_GPIO_Port,Cerpadlo_posun1_Pin,cerpadlo1);
	  HAL_GPIO_WritePin(Cerpadlo_posun2_GPIO_Port,Cerpadlo_posun2_Pin,cerpadlo2);
  }
}

int speed_cerpadla;
int timeout;
void start_cerpani_v_nadrzi(int time){
	HAL_GPIO_WritePin(cerpadlo_interni_GPIO_Port,cerpadlo_interni_Pin,GPIO_PIN_SET);
	timeout = HAL_GetTick()+(time*100);
	while(HAL_GetTick()<timeout){
	}
	HAL_GPIO_WritePin(cerpadlo_interni_GPIO_Port,cerpadlo_interni_Pin,GPIO_PIN_RESET);
}
void start_cerpani_v_rameni(int time){
	HAL_GPIO_WritePin(cerpadlo_externi_GPIO_Port,cerpadlo_externi_Pin,GPIO_PIN_SET);
	timeout = HAL_GetTick()+(time*100);
	while(HAL_GetTick()<timeout){
	}
	HAL_GPIO_WritePin(cerpadlo_externi_GPIO_Port,cerpadlo_externi_Pin,GPIO_PIN_RESET);
}


void set_speed_cerpadla(int speed_cerpadla){
	if(speed_cerpadla>0){
		cerpadlo1 = GPIO_PIN_SET;
		cerpadlo2 = GPIO_PIN_RESET;
	}
	if(speed_cerpadla<0){
		cerpadlo1 = GPIO_PIN_RESET;
		cerpadlo2 = GPIO_PIN_SET;
	}
	if(speed_cerpadla==0){
		cerpadlo1 = GPIO_PIN_RESET;
		cerpadlo2 = GPIO_PIN_RESET;
	}
}

int tempdistance;
void posun_cerpadla(uint8_t smer)
{
	if (smer == nahoru){
		distance_cerpadlo = 0;
		speed_cerpadla = max_rychlost_cerpadla;
		set_speed_cerpadla(speed_cerpadla);
		tempdistance = 0;
		timeout = 0;
		while (1){
			if (tempdistance!=distance_cerpadlo){
				timeout = HAL_GetTick()+cerpadlo_timeout;
				tempdistance = distance_cerpadlo;
			}
			if ((timeout!=0)&(timeout<HAL_GetTick())){
				break;
			}
		}
		speed_cerpadla = 0;
		set_speed_cerpadla(speed_cerpadla);

	}
	if (smer == dolu){
		distance_cerpadlo = 0;
		speed_cerpadla = -max_rychlost_cerpadla;
		set_speed_cerpadla(speed_cerpadla);
		tempdistance = 0;
		timeout = 0;
		while (1){
			if (tempdistance!=distance_cerpadlo){
				timeout = HAL_GetTick()+cerpadlo_timeout;
				tempdistance = distance_cerpadlo;
			}
			if ((timeout!=0)&(timeout<HAL_GetTick())){
				break;
			}
		}
		speed_cerpadla = 0;
		set_speed_cerpadla(speed_cerpadla);

	}
}
void start_cerpadlo_control(){
	speed_cerpadla = 0;
	cerpadlo1 = GPIO_PIN_RESET;
	cerpadlo2 = GPIO_PIN_RESET;
}
void reset_poss_cerpadla(void){
	distance_cerpadlo = 0;
}

#endif /* CERPADLO_C_ */
