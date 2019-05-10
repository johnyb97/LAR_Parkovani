/*
 * Cerpadlo.h
 *
 *  Created on: Apr 29, 2019
 *      Author: janbe
 */

#ifndef CERPADLO_H_
#define CERPADLO_H_

#include "main.h"
#define nahoru 1
#define dolu 0
#define max_rychlost_cerpadla  9
#define cerpadlo_timeout 100

int distance_cerpadlo; //poèet pootoèení o úhel encoderu...pozor bude se lišit pøi zapojení všech 4 enkodérù a jen 2...

void reset_poss_cerpadla(void);
void posun_cerpadla(uint8_t smer);
void start_cerpadlo_control(void);
void start_cerpani_v_nadrzi(int time);
void start_cerpani_v_rameni(int time);

#endif /* CERPADLO_H_ */
