/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * Nota de Edición: 
  * Según información leída, "se aplica una entrada escalón entre 10% y 20% del valor nominal y se registra la respuesta de salida hasta que se estabilice en el nuevo punto de operación." Esto con el fin de determinar correctamente la función transferencia.
  * De esto se obtiene una planta con 78,59% de aproximación; superando considerablemente al valor de 65% obtenido con los valores de PWM anteriores.
  * Se obtiene la siguiente planta:
  * G(s) = Kp/(1+Tp1*s)
  * Kp = 0.1026
  * Tp1 = 0.13677  
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "pid.h"
#include "communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//Constantes de la planta (creo que no son necesarias)
#define K0 0.1026 //Ganancia de la planta
#define gamma0 0.13677 //Tiempo de la planta

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//Registro ADC
uint16_t suma_ADC_values = 0;
uint16_t ADC_register [N_FILTRO_FIR] = {0}; //almacena los ultimos 32 datos del ADC
uint16_t ADC_IN = 0;

//Acondicionamiento de señal
float ADC_to_V = 0;

uint8_t R1 = 100; //resistencia de 100k
uint8_t R2 = 100; //resistencia de 100k
uint8_t PWM_percent = 0;

//Comunicación
bool flag_adc_read = 0;

instrucciones_t estado_actual = STOP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void SwitchMode(instrucciones_t mode){
  estado_actual = mode;

  switch(estado_actual){
    case STOP:
      snprintf(bufferTx, sizeof(bufferTx), "Detenido...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case WAIT:
      snprintf(bufferTx, sizeof(bufferTx), "Esperando...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case ROUTINE: 
      snprintf(bufferTx, sizeof(bufferTx), "Ejecución de Rutina...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case PID_PARAMETERS_READ:
      snprintf(bufferTx, sizeof(bufferTx), "Leyendo parámetros PID...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      snprintf(bufferTx, sizeof(bufferTx), "Kp_z: %.3f\r\nKi_z: %.3f\r\nKd_z: %.3f\r\nTs: %.3f",PID.Kp_z,PID.Ki_z,PID.Kd_z,PID.Ts);
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case PID_SET_POINT_READ:
      snprintf(bufferTx, sizeof(bufferTx), "Set Point: %.1f\r\n",PID.Maths.set_point);
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case PID_SET_POINT_SET:
      snprintf(bufferTx, sizeof(bufferTx), "Modificando Set Point...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case PID_PARAMETERS_SET:
      snprintf(bufferTx, sizeof(bufferTx), "Modificando parametros del PID...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case PID_START:
      snprintf(bufferTx, sizeof(bufferTx), "Iniciando ejecución...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;

    case CLEAR_MEMORY:
      snprintf(bufferTx, sizeof(bufferTx), "Borrando memoria...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;
    
    case PID_PARAMETERS_RESET:
      snprintf(bufferTx, sizeof(bufferTx), "Reiniciando variables del PID...\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      break;
  }
}

void ProccessBufferRx(char* bufferRx){
  if (strncmp(bufferRx, "stop", 4) == 0){
    SwitchMode(STOP);
  }
  else if (strncmp(bufferRx, "wait", 4) == 0){
    SwitchMode(WAIT);
  }
  else if (strncmp(bufferRx, "routine", 7) == 0){
    SwitchMode(ROUTINE);
  }
  else if (strncmp(bufferRx, "clear", 5) == 0){
    SwitchMode(CLEAR_MEMORY);
  }
  else if (strncmp(bufferRx, "pid reset", 9) == 0){
    SwitchMode(PID_PARAMETERS_RESET);
  }
  else if (strncmp(bufferRx, "pid parameters", 14) == 0){
    SwitchMode(PID_PARAMETERS_READ);
  }
  else if (strncmp(bufferRx, "set point", 9) == 0){
    SwitchMode(PID_SET_POINT_READ);
  }

  else if (strncmp(bufferRx, "SP: ", 3) == 0){
    SwitchMode(PID_SET_POINT_SET);
  }

  else if (strncmp(bufferRx,"Kp: ", 3) == 0 || strncmp(bufferRx,"Ki: ", 3) == 0 || strncmp(bufferRx,"Kp: ", 3) == 0){
    SwitchMode(PID_PARAMETERS_SET);
  }

  else if (strncmp(bufferRx, "start", 5) == 0){
    SwitchMode(PID_START);
  }
  
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//ADC
  if(htim->Instance == TIM4){

    HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1,100) == HAL_OK){
      ADC_IN = HAL_ADC_GetValue(&hadc1);  //obtengo el valor actual

      suma_ADC_values -= ADC_register[Data_mem.cont_tim4 & (N_FILTRO_FIR - 1)];
      ADC_register[Data_mem.cont_tim4 & (N_FILTRO_FIR - 1)] = ADC_IN;
      suma_ADC_values += ADC_register[Data_mem.cont_tim4 & (N_FILTRO_FIR - 1)];
     
      //acondicionamiento de señal
      ADC_to_V =3.3 * (1+R1/R2) * (suma_ADC_values >> BITS_FILTRO_FIR)/1023.0;

      //Registro de datos en memoria
      if (Data_mem.cont_tim4 < TAMAÑO_MEMORIA_DATOS){
        Data_mem.PWM [Data_mem.cont_tim4] = PWM_percent;
        Data_mem.ADC_to_V [Data_mem.cont_tim4] = ADC_to_V;
        Data_mem.set_point [Data_mem.cont_tim4] = PID.Maths.set_point;
        Data_mem.cont_tim4++;
      }
	  }
	  
    flag_adc_read = true;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  PIDCreate(Kp,Ki,Kd,Tss);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  //state = wait;
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //apago led  
  //TIM4->DIER &= ~TIM_DIER_UIE; //deshabilito interrupciones de timer4

  //HAL_Delay(2000); //tiempo de conexión PC/micro
  
  //esto se deberia poder sacar
  ClearMemory();
  PIDReset(); 

  //state = start;
  //TIM4->DIER |= TIM_DIER_UIE; //habilito interrupciones de timer4
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //enciendo led
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
    
	  // RECEPCION DE DATOS por usb
    if (flag_bufferRx){
      flag_bufferRx = 0;
      ProccessBufferRx(bufferRx);
    }

    // Estados
    if (estado_actual == STOP){
      SetSetPoint(0);
      TIM4->DIER &= ~TIM_DIER_UIE; //deshabilito interrupciones de timer4
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //apago led
    }

    if (estado_actual == WAIT){
      
    }

    if (estado_actual == ROUTINE){
      TIM4->DIER |= TIM_DIER_UIE; //habilito interrupciones de timer4
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //enciendo led

      //Rutina
      if (Data_mem.cont_tim4 == 500)  SetSetPoint(1) ;
      if (Data_mem.cont_tim4 == 1500) SetSetPoint(3);
      if (Data_mem.cont_tim4 == 2000) SetSetPoint(1.8);
      if (Data_mem.cont_tim4 == 2500) SetSetPoint(0);
      if (Data_mem.cont_tim4 >= 3000){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //apago led
        TIM4->DIER &= ~TIM_DIER_UIE; //deshabilito interrupciones de timer4
        
        PrintConsoleMemory(); //imprimo en consola
        SwitchMode(STOP);
      };
    }

    if (estado_actual == PID_START){
      TIM4->DIER |= TIM_DIER_UIE; //habilito interrupciones de timer4
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //enciendo led
    }

    if (estado_actual == PID_SET_POINT_SET){
      char* ptr = bufferRx + 3; //apunto al valor del set point
      float set_point = strtof(ptr, NULL); //convierto el string a float
      SetSetPoint(set_point); //seteo el set point
      snprintf(bufferTx, sizeof(bufferTx), "Set Point: %.1f\r\n", PID.Maths.set_point);
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      SwitchMode(WAIT);
    }

    if (estado_actual == PID_PARAMETERS_SET){
      char* ptr = bufferRx + 3; //apunto al valor del set point
      float value = strtof(ptr, NULL); //convierto el string a float

      //Filtro de constante
      if (strncmp(bufferRx, "Kp: ", 3) == 0){
        value = value;  //constante Kp
        SetPIDConst(&PID.Kp_z, value);
        snprintf(bufferTx, sizeof(bufferTx), "Kp_z seteado a: %.3f\r\n", PID.Kp_z);
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      }
      if (strncmp(bufferRx, "Ki: ", 3) == 0){
        value = value * PID.Ts; //constante Ki
        SetPIDConst(&PID.Ki_z, value);
        snprintf(bufferTx, sizeof(bufferTx), "Ki_z seteado a: %.3f\r\n", PID.Ki_z);
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      }
      if (strncmp(bufferRx, "Kd: ", 3) == 0){
        value = value / PID.Ts; //constante Ki
        SetPIDConst(&PID.Kd_z, value);
        snprintf(bufferTx, sizeof(bufferTx), "Kd_z seteado a: %.3f\r\n", PID.Kd_z);
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      }

      SwitchMode(WAIT);
    }

    if(estado_actual == CLEAR_MEMORY){
      ClearMemory();
      snprintf(bufferTx, sizeof(bufferTx), "Memoria de datos borrada.\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      SwitchMode(WAIT);
    }
    if(estado_actual == PID_PARAMETERS_RESET){
      SetSetPoint(0);
      PIDInit(Kp,Ki,Kd,Tss);
      snprintf(bufferTx, sizeof(bufferTx), "Se reiniciaron las variables del PID.\r\n");
      CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
      SwitchMode(WAIT);
    }
    
    

    if (flag_adc_read == true){
      PWM_percent = PIDRefresh(ADC_to_V);
      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,PWM_percent*10);

      //Imprimir en consola info relevante
      if(estado_actual != ROUTINE){
        snprintf(bufferTx, sizeof(bufferTx), "PWM: %u - V_tac: %.2f\r\n", PWM_percent,ADC_to_V);
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      }

      //limitador de error acumulado
      if (PID.Maths.integral > 1000){
        SetSetPoint(0);
        SwitchMode(WAIT);
        snprintf(bufferTx, sizeof(bufferTx), "Error de tensión. Reiniciar sistema!\r");
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        break;
      }

      flag_adc_read = false;
    }
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(enable_read_GPIO_Port, enable_read_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : enable_read_Pin */
  GPIO_InitStruct.Pin = enable_read_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(enable_read_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypedef *htim){
//	if (htim->Instance == TIM4){
//		HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
