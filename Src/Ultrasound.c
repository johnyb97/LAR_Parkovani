/*
 * Ultrasound.c
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#include "Ultrasound.h"
#include "motorcontrol.h"

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	if (htim->Instance == TIM22){
		if (logic1!=1){
			dist1 = (dist1 + 65000)/2;
		}else{
			logic1 =0;
		}
		if (logic2!=1){
			dist2 = (dist2 + 65000)/2;
		}else{
			logic2 =0;
		}
	}

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PWM_PulseFinishedCallback could be implemented in the user file
   */
}
void init_ultrasound(TIM_HandleTypeDef *htim){
    dist1 = 2000;
    logic1 = 0;
    dist2 = 2000;
    logic2 = 0;
    HAL_TIM_PWM_Init(htim);
    HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
    TIM_ultrasound = htim;
}
int get_ultrasound1_dist(void){
	return dist1;
}
int get_ultrasound2_dist(void){
	return dist2;
}

