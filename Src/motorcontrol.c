/*
 * motorcontrol.c
 *
 *  Created on: Mar 13, 2019
 *      Author: janbe
 */
#include "motorcontrol.h"
#include "Ultrasound.h"
#include "main.h"
#include "Bluetooth.h"

int set_speed_rigth;
int set_speed_left;
int16_t speed1;
int16_t speed2;
int16_t speed3;
int16_t speed4;

int16_t speed_cerpadlo_nahoru;
int16_t speed_cerpadlo_dolu;

uint16_t chan1; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak
uint16_t chan2; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak

int response;
uint8_t a;
uint8_t b;
int Kp;
float Ki;
int	Kd;
float Integral;
float LastError;
float Derivate;
int pow1;
int pow2;
int otacky1;
int otacky2;
float error;
float error2;
float Turn;
uint32_t timer;
uint32_t timer2;
float Kp2;
float Ki2;
float	Kd2;
float Integral2;
float LastError2;
float Derivate2;
int etalon;

uint8_t stop;
uint8_t total_stop;
int logic;
int rigth; //speed of rigth engine
int left; //speed of left engine
int distance;
int last_distance_traveled;
float maxdistance;
int currentdistance;
int currentdistance1;
int currentdistance2;

void set_stop(uint8_t valuestop,uint8_t valuetotal_stop){
	stop = valuestop;
	total_stop=valuetotal_stop;
}

void PID_podle_ultrazvuku(TIM_HandleTypeDef *htim)
{
	if (timer2<HAL_GetTick()){
			error2 = (etalon-get_ultrasound1_dist());
			Integral2 = Integral2 + error2;
			Derivate2 = error2-LastError2;
			Turn = (Kp2 * error2+Ki2*Integral2 + Kd2*Derivate2)/500;
			pow1 = saturation(set_speed_rigth +Turn);
			pow2 = saturation(set_speed_left-Turn);
			LastError2 = error2;
			set_motor_speed(htim,pow1,pow2);
			timer2= HAL_GetTick()+10; //frequency of PID regulator cca 10ms;
			error2=(error2/200);
			send_bluetooth((uint8_t*)(&error2));
		}

}

void Parkovani(TIM_HandleTypeDef *htim,int power){
	dira=0;
	rigth=power;
	left=power;
	set_speed(htim,left,rigth);
	 while (1)
		  {
		 	 if ((dist1<(etalon+200)) && (dira==0))
				{
		 		PID_podle_ultrazvuku(htim);
				}

			 //PID_podle_ultrazvuku(&htim2); //call PID regulator with its own frequency 100Hz

			  if ((dist1>(etalon+200))&&(dist1<1500)&&(dira==0))
			  	 			  {
			  	 				 dira=1; //signalizac ze projel kolem diry
			  	 				 ujeto=distance_traveled(1); //hodnota ujete vzdalenosti
			  	 				set_speed(htim,left,rigth);
			  	 			  }
			  if ((dist1>(etalon+200))&&(dira=1))
			  	 			  {
				  	  	  	  ujeto=distance_traveled(1)-ujeto; //
				  	  	  	  break;
			  	 			  }
		  	  	  }
	 	 while(1)
	 	 {
	 		PID_podle_ultrazvuku(&htim);
	 		if(distance_traveled(1)>(ujeto+200))
	 		{
	 			break;
	 		}
	 	 }
	 Travel_rovne(htim,(ujeto/2)+200,500);
	 Travel_turn(htim,90,500);
	 Travel_rovne(htim,700,500);
}

void start_motor_control(TIM_HandleTypeDef *htim){
	speed1 = 0;
	speed2 = 0;
	speed3 = 0;
	speed4 = 0;
	speed_cerpadlo_nahoru = 0;
	speed_cerpadlo_dolu = 0;
	HAL_TIM_PWM_Init(htim);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_4);
	TIM3->CCR1 = speed1;
    TIM3->CCR2 = speed2;
    TIM3->CCR3 = speed3;
    TIM3->CCR4 = speed4;
    set_stop(0,0);
}
void set_speed(TIM_HandleTypeDef *htim,int speedrigth,int speedleft){
	set_speed_rigth = speedrigth;
	set_speed_left = speedleft;
	set_motor_speed(htim,set_speed_rigth,set_speed_left);
}

int set_motor_speed(TIM_HandleTypeDef *htim,int speedrigth,int speedleft){
	if (speedrigth == 0){
		speed1 = 0;
		speed2 = 0;
	}
	if (speedleft ==0){
		speed3 = 0;
		speed4 = 0;
	}
	if(speedrigth >0){
		speed1 = speedrigth;
	    speed2 = 0;
	}
	if (speedleft >0){
	    speed3 = speedleft;
		speed4 = 0;
	}
	if (speedrigth <0){
		speed1 = 0;
	    speed2 = (-speedrigth);
	}
	if (speedleft<0){
	    speed3 = 0;
	    speed4 = (-speedleft);
	}
	TIM3->CCR1 = speed2;
	TIM3->CCR2 = speed1;
	TIM3->CCR3 = speed4;
	TIM3->CCR4 = speed3;
	timer= HAL_GetTick()+10; //gives time for engines to start working
	return 1; //set
}
/*
 * Tohle je tøeba dodìlat...zbytek snad bude fungovat...
 */
void PID_regularor(TIM_HandleTypeDef *htim){
	if (timer<HAL_GetTick()){
		error = (distance1-distance2);
		Integral = Integral + error;
		Derivate = error-LastError;
		Turn = (Kp * error+Ki*Integral + Kd*Derivate)/30;
		pow1 = saturation(set_speed_rigth+Turn);
		pow2 = saturation(set_speed_left-Turn);
		LastError = error;
		set_motor_speed(htim,pow1,pow2);
		timer= HAL_GetTick()+10; //frequency of PID regulator cca 10ms;
	}
}


int Travel_rovne(TIM_HandleTypeDef *htim, int vzdalenost,int power){
	  rigth = power; //speed of rigth engine
	  left = power; //speed of left engine
	  set_speed(htim,left,rigth);
	  maxdistance =1560*vzdalenost/100;
	  while (1)
	  {

				 PID_regularor(htim); //call PID regulator with its own frequency 100Hz

				 if (distance_traveled(2)>(last_distance_traveled+30))//korekce staceni
						  {
						  distance2=distance2+1;
						  last_distance_traveled=distance_traveled(2);
						  }

			  if (stop==1)
							  {
								 rigth = 0; //speed of rigth engine
								 left = 0; //speed of left engine
								 logic=1;
								 set_speed(htim,left,rigth);
							  }
				 if (stop==2)
							  {
								  rigth = power; //speed of rigth engine
								  left = power; //speed of left engine
								  logic=0;
								  set_speed(htim,left,rigth);
							  }
		 if (power>0){
			 if (total_stop==1)
				 {
					rigth = 0; //speed of rigth engine
					left = 0; //speed of left engine

						set_speed(htim,left,rigth);
									  distance=distance_traveled(1);
												reset_distance();
												return distance;
				 }
		 }

		if (distance_traveled(1)>maxdistance)


			 {
				 rigth = 0; //speed of rigth engine
				 left = 0; //speed of left engine

				  set_speed(htim,rigth,left);
				  distance=distance_traveled(1);
				  			 reset_distance();
				  			return distance;


			 }


	  }
}


int Motor_left(TIM_HandleTypeDef *htim,int vzdalenost,int power){
	  left = power; //speed of left engine
	  set_speed(htim,left,0);
	  maxdistance =1587*vzdalenost/100;
	  while (1)
	  {

		if (distance_traveled(2)>maxdistance)


			 {
				 rigth = 0; //speed of rigth engine
				 left = 0; //speed of left engine

				 set_speed(htim,left,rigth);
				  distance=distance_traveled(2);
				  			 reset_distance();
				  			return distance;


			 }


	  }
}

int Motor_right(TIM_HandleTypeDef *htim,int vzdalenost,int power){
	  //right = power; //speed of left engine
	  set_speed(htim,0,power);
	  maxdistance =1587*vzdalenost/100;
	  while (1)
	  {

		if (distance_traveled(1)>maxdistance)


			 {
				 rigth = 0; //speed of rigth engine
				 left = 0; //speed of left engine

				  set_speed(htim,rigth,left);
				  distance=distance_traveled(1);
				  			 reset_distance();
				  			return distance;


			 }


	  }
}

int Motor_straigt(TIM_HandleTypeDef *htim,int vzdalenost,int power){
	  rigth = power; //speed of left engine
	  last_distance_traveled=0;
	  left= power;
	  set_speed(htim,left,rigth);
	  maxdistance =1587*vzdalenost/100;
	  while (1)
	  {
		  if (distance_traveled(1)>(last_distance_traveled+24))//korekce staceni
		  {
		  distance1=distance1+1;
		  last_distance_traveled=distance_traveled(1);
		  }

		if (distance_traveled(1)>maxdistance)


			 {
				 rigth = 0; //speed of rigth engine
				 set_speed(htim,left,rigth);
			 }
		if (distance_traveled(2)>maxdistance)


					 {
						 left = 0; //speed of rigth engine
						 set_speed(htim,left,rigth);
					 }

		if ((distance_traveled(1)>maxdistance) && (distance_traveled(2)>maxdistance))
		{
			rigth = 0;
			left = 0;
			set_speed(htim,left,rigth);
				  distance=distance_traveled(1);
				  wait(5000);
				  			 reset_distance();
				  			return distance;
		}


			 }


	  }


int Travel_turn(TIM_HandleTypeDef *htim2,int uhel,int power){
	if (uhel>=0){
		rigth = power; //speed of rigth engine
		left =-power; //speed of left engine
	}else{
		rigth = -power; //speed of rigth engine
		left = power; //speed of left engine
	}
	set_speed(htim2,rigth,left);
	if (uhel>=0){
		maxdistance = 310*uhel/90;
	}else{
		maxdistance = 310*(-uhel)/90;
	}
	while (1)
	{
		if (stop==1)
		{
		  	 rigth = 0; //speed of rigth engine
		  	 left = 0; //speed of left engine
		  	 logic=1;
		  	 set_speed(htim2,rigth,left);
		}
		if (stop==2)
		{
		  	 rigth = power; //speed of rigth engine
		  	 left = -power; //speed of left engine
		  	 logic=0;
		  	 stop = 0;
		  	 set_speed(htim2,rigth,left);
		 }
		if (total_stop==1)
			 {
				rigth = 0; //speed of rigth engine
				left = 0; //speed of left engine
				set_speed(htim2,rigth,left);
				distance=distance_traveled(1);
				reset_distance();
				return distance;
			 }
		 currentdistance1 = distance_traveled(1);
		 currentdistance2 = distance_traveled(2);
		 if (currentdistance1>currentdistance2){
			 currentdistance = currentdistance1;
		 }else{
			 currentdistance = currentdistance2;
		 }


		 if (currentdistance>maxdistance)
		 {
		    rigth = 0; //speed of rigth engine
			left = 0; //speed of left engine
			set_speed(htim2,rigth,left);
			distance=distance_traveled((int)1);
			reset_distance();
			return distance;
		}
	}
}

int saturation(int variable){
	if (variable>1000){
		variable=1000;
    }
	if (variable<0 || variable>90000){
		variable=0;
	}
	return variable;
}

void init_PID(void){
	chan1 = 0; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak
	chan2 = 0; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak
	Kp = 400;
	Ki = 0.03;
	Kd = 10;
	Integral = 0;
	LastError = 0;
	Derivate = 0;
    timer = 0;
}

void init_PID_ultrazvuk(void){
	chan1 = 0; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak
	chan2 = 0; //cas za jaky se posunulo kolo o úhel...pùjde zpøesnit když pøidáme 4 kanály ale zatím snad staèí tak
	Kp2 = 400;
//Ki2 = 0.02;
	Ki2 = 0;
	Kd2 = 0;
	Integral2 = 0;
	LastError2 = 0;
	Derivate2 = 0;
    timer2 = 0;
    etalon=1300;
}

void start_encoder(void){
    distance1 = 0;
    distance2 = 0;
}
void reset_distance(void){
	distance1 = 0;
	distance2 = 0;
}
int distance_traveled(int wheel){
	if (wheel ==1){
		return distance1;
	}
	if (wheel ==2){
	   return distance2;
	}
	return distance1;
}
