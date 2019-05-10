/*
 * External_interupt.c
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#include "Ultrasound.h"
#include "motorcontrol.h"
#include "main.h"
#include "Cerpadlo.h"
int tmparr[255];
int i;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if (GPIO_Pin == Utrasound_sens1_Pin){
			if (HAL_GPIO_ReadPin(Utrasound_sens1_GPIO_Port, Utrasound_sens1_Pin)){
				//tempUltra=TIM_ultrasound->Instance->CNT;
				tempUltra=TIM22->CNT;
				logic1=1;

			}

			else {
				if (logic1==1)
				{

				dist1 = (((TIM22->CNT)-tempUltra));
											//TIM_ultrasound->Instance->CNT=0;
											//htim22.Instance->CNT = 0;
											logic1 =0;
				i = i%255;
				tmparr[i] =dist1;
				i++;
				}

			}
		}

	if (GPIO_Pin == Utrasound_sens2_Pin){
			if (HAL_GPIO_ReadPin(Utrasound_sens2_GPIO_Port, Utrasound_sens2_Pin) == GPIO_PIN_RESET){
				//dist2 = (dist2 + TIM_ultrasound->Instance->CNT)/2;
				//htim22.Instance->CNT = 0;
				logic2 =1;
			}
		}
	/*
	if (GPIO_Pin == EncoderRigth1_Pin){
		if (HAL_GPIO_ReadPin(EncoderRigth1_GPIO_Port, EncoderRigth1_Pin) == GPIO_PIN_SET){
			distance2++; //posun o daný úhel
		}
	}
	if (GPIO_Pin == EncoderLeft1_Pin){
		if (HAL_GPIO_ReadPin(EncoderLeft1_GPIO_Port, EncoderLeft1_Pin) == GPIO_PIN_SET){
			distance1++; //posun o daný úhel
		}
	}
	*/
	if (GPIO_Pin == Encoder_cerpadlo_Pin){
			if (HAL_GPIO_ReadPin(Encoder_cerpadlo_GPIO_Port, Encoder_cerpadlo_Pin) == GPIO_PIN_SET){
				distance_cerpadlo++; //posun o daný úhel
		}
	}
	/*
	if (GPIO_Pin == Mikrofon1_Pin){
			if (HAL_GPIO_ReadPin(EncoderLeft1_GPIO_Port, EncoderLeft1_Pin) == GPIO_PIN_SET){

			}
		}
	/*
	if (GPIO_Pin == Dotek_senzor_Pin){
		if (HAL_GPIO_ReadPin(Dotek_senzor_GPIO_Port, Dotek_senzor_Pin) == GPIO_PIN_SET){
			set_stop(0,1);
		}
	}
	*/
}
