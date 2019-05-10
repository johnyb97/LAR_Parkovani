/*
 * Ultrasound.h
 *
 *  Created on: Apr 5, 2019
 *      Author: janbe
 */

#ifndef ULTRASOUND_H_
#define ULTRASOUND_H_

#include "main.h"
uint8_t logic1;
int tempUltra;
uint8_t logic2;
int dist1;
int dist2;
TIM_HandleTypeDef* TIM_ultrasound;
void init_ultrasound(TIM_HandleTypeDef *htim);
int get_ultrasound1_dist(void);
int get_ultrasound2_dist(void);

#endif /* ULTRASOUND_H_ */
