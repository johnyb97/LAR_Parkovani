/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWMmotorRight1_Pin GPIO_PIN_6
#define PWMmotorRight1_GPIO_Port GPIOA
#define PWMmotorRight2_Pin GPIO_PIN_7
#define PWMmotorRight2_GPIO_Port GPIOA
#define Utrasound_sens2_Pin GPIO_PIN_4
#define Utrasound_sens2_GPIO_Port GPIOC
#define Utrasound_sens2_EXTI_IRQn EXTI4_15_IRQn
#define Cerpadlo_posun1_Pin GPIO_PIN_5
#define Cerpadlo_posun1_GPIO_Port GPIOC
#define PWMmotorLeft1_Pin GPIO_PIN_0
#define PWMmotorLeft1_GPIO_Port GPIOB
#define PWMmotorLeft2_Pin GPIO_PIN_1
#define PWMmotorLeft2_GPIO_Port GPIOB
#define Encoder_cerpadlo_Pin GPIO_PIN_10
#define Encoder_cerpadlo_GPIO_Port GPIOB
#define Encoder_cerpadlo_EXTI_IRQn EXTI4_15_IRQn
#define Mikrofon3_Pin GPIO_PIN_11
#define Mikrofon3_GPIO_Port GPIOB
#define Mikrofon3_EXTI_IRQn EXTI4_15_IRQn
#define EncoderLeft2_Pin GPIO_PIN_12
#define EncoderLeft2_GPIO_Port GPIOB
#define EncoderLeft2_EXTI_IRQn EXTI4_15_IRQn
#define EncoderRigth2_Pin GPIO_PIN_13
#define EncoderRigth2_GPIO_Port GPIOB
#define EncoderRigth2_EXTI_IRQn EXTI4_15_IRQn
#define EncoderLeft1_Pin GPIO_PIN_14
#define EncoderLeft1_GPIO_Port GPIOB
#define EncoderLeft1_EXTI_IRQn EXTI4_15_IRQn
#define EncoderRigth1_Pin GPIO_PIN_15
#define EncoderRigth1_GPIO_Port GPIOB
#define EncoderRigth1_EXTI_IRQn EXTI4_15_IRQn
#define Mikrofon1_Pin GPIO_PIN_7
#define Mikrofon1_GPIO_Port GPIOC
#define Mikrofon1_EXTI_IRQn EXTI4_15_IRQn
#define cerpadlo_interni_Pin GPIO_PIN_8
#define cerpadlo_interni_GPIO_Port GPIOC
#define Dotek_senzor_Pin GPIO_PIN_9
#define Dotek_senzor_GPIO_Port GPIOC
#define Dotek_senzor_EXTI_IRQn EXTI4_15_IRQn
#define Utrasound_sens1_Pin GPIO_PIN_8
#define Utrasound_sens1_GPIO_Port GPIOA
#define Utrasound_sens1_EXTI_IRQn EXTI4_15_IRQn
#define Cerpadlo_posun2_Pin GPIO_PIN_12
#define Cerpadlo_posun2_GPIO_Port GPIOA
#define Ultrasound_PWM_Pin GPIO_PIN_4
#define Ultrasound_PWM_GPIO_Port GPIOB
#define cerpadlo_externi_Pin GPIO_PIN_5
#define cerpadlo_externi_GPIO_Port GPIOB
#define STEP4_Pin GPIO_PIN_6
#define STEP4_GPIO_Port GPIOB
#define STEP1_Pin GPIO_PIN_7
#define STEP1_GPIO_Port GPIOB
#define STEP3_Pin GPIO_PIN_8
#define STEP3_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_9
#define STEP2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
