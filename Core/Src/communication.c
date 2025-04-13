/**
 * @file communication.c
 * @author Joel Jassan <joeljassan@hotmail.com>
 * @brief  Funciones de comunicación de información serial del proyecto
 * @date 2025-04
 *
 * @copyright Copyright (c) 2025. All rights reserved.
 *
 */
/* --------------------------------------------------------------------------------------------- */

/*---  Includes  ------------------------------------------------------------------------------- */
#include "communication.h"

/*---  Macros and Definitions  ----------------------------------------------------------------- */

/*---  Private Data Declaration  --------------------------------------------------------------- */

/*---  Public Data Declaration  ---------------------------------------------------------------- */
Data_mem_t Data_mem = {0};

/*---  Private Function Declaration  ----------------------------------------------------------- */

/*---  Public Function Declaration  ------------------------------------------------------------ */

/*---  Private Data Definition  ---------------------------------------------------------------- */

/*---  Public Data Definition  ----------------------------------------------------------------- */

char bufferRx[64] = {0};
char bufferTx[64] = {0};
bool flag_bufferRx;

/*---  Private Function Definition  ------------------------------------------------------------ */

/*---  Public Function Definition  ------------------------------------------------------------- */

/*---  Private Function Implementation  -------------------------------------------------------- */

/*---  Public Function Implementation  --------------------------------------------------------- */

/**
 * @brief Limpia la memoria de datos
 *
 */
void ClearMemory(void){
    // Limpia los arrays de floats
    for (int i = 0; i < TAMAÑO_MEMORIA_DATOS; i++) {
      Data_mem.ADC_to_V[i] = 0.0f;
      Data_mem.set_point[i] = 0.0f;
      Data_mem.PWM[i] = 0;
    } //se hace un loop for para limpiar la memoria porque recomiendan que se haga de esta forma cuando son variables float
  
    // Reinicia el contador
    Data_mem.cont_tim4 = 0;
  }

  /**
   * @brief Imprime los titulos de la tabla en consola
   * 
   */
void PrintTableConsole(void){
    snprintf(bufferTx, sizeof(bufferTx), "ADC Read (10 bits)\r\n");
    CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
    snprintf(bufferTx, sizeof(bufferTx), "Muestra\t PWM_percent\t Set_Point\t ADC_to_V\r\n");
    CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx));
}

/**
 * @brief Imprime los datos de la memoria en consola
 * 
 * @note Esta funcion llama a PrintTableConsole()
 */
void PrintConsoleMemory(void){
    PrintTableConsole();

    for(int i = 0; i < Data_mem.cont_tim4; i++){
        if (0 == (i & 0x1F)) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //parpadeo led
    
        snprintf(bufferTx, sizeof(bufferTx), "%d,%u,%.2f,%.2f\r\n", i, Data_mem.PWM[i], Data_mem.set_point[i], Data_mem.ADC_to_V[i]);
        CDC_Transmit_FS((uint8_t*)bufferTx, strlen(bufferTx)); //imprimo en consola
      }
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //apago led
}

/*---  End of File  ---------------------------------------------------------------------------- */
