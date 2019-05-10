/*
 * Step_eng.c
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#include "Step_eng.h"

void init_Step(void){
	step_poss = 0;
	Overall_poss = 0;
	timeout =0;
}

void next_step(void){
	if (timeout<HAL_GetTick()){
		switch(step_poss){
		   case 0:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_SET);
		   break;
		   case 1:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_SET);
		   break;
		   case 2:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
		   case 3:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
		   case 4:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
		   case 5:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
			 case 6:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
		   case 7:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_SET);
		   break;
		   default:
			 HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP2_GPIO_Port,STEP2_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP3_GPIO_Port,STEP3_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(STEP4_GPIO_Port,STEP4_Pin, GPIO_PIN_RESET);
		   break;
		}
		step_poss++;
		if (step_poss>=8){
			step_poss = 0;
		}
	timeout = HAL_GetTick()+1;   ;
	}
}
