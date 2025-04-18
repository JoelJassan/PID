/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

//Constantes del PID, en el dominio S
#define Kp 23
#define Ki 315
#define Kd 0
#define Tss 0.001 //Tiempo de muestreo

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define pwm_Pin GPIO_PIN_6
#define pwm_GPIO_Port GPIOA
#define enable_read_Pin GPIO_PIN_12
#define enable_read_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef enum {
  STOP,
  WAIT,
  PID_PARAMETERS_SET,
  PID_PARAMETERS_READ,
  PID_PARAMETERS_RESET,
  PID_SET_POINT_SET,
  PID_SET_POINT_READ,
  PID_START,
  ROUTINE,
  CLEAR_MEMORY,
} instrucciones_t;

extern instrucciones_t estado_actual;


void SwitchMode(instrucciones_t mode);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
