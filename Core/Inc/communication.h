/**
 * @file communication.h
 * @author Joel Jassan <joeljassan@hotmail.com>
 * @brief  Declaracion de datos para comunicacion serial del proyecto
 * @date 2025-04
 *
 * @copyright Copyright (c) 2025. All rights reserved.
 *
 */
/* --------------------------------------------------------------------------------------------- */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/*---  Includes  ------------------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "usbd_cdc_if.h"


/*---  Macros  --------------------------------------------------------------------------------- */

//Filtro FIR de ruido
#define N_FILTRO_FIR 32
#define BITS_FILTRO_FIR (__builtin_ctz(N_FILTRO_FIR))

// Registro de datos de ADC filtrado
#define TAMAÑO_MEMORIA_DATOS 4000

/*---  Definitions  ---------------------------------------------------------------------------- */

/*---  Public Data Declaration  ---------------------------------------------------------------- */

/*
 * @brief Estructura con los datos del registro de la muestra de datos
 */
typedef struct Data_mem_s
{
    float ADC_to_V[TAMAÑO_MEMORIA_DATOS]; /**< almacena el valor de ADC filtrado y ajustado a V */
    float set_point[TAMAÑO_MEMORIA_DATOS];/**< almacena el valor de set point */
    int cont_tim4;
    uint8_t PWM[TAMAÑO_MEMORIA_DATOS]; /**< almacena el valor de PWM */
} Data_mem_t;

extern Data_mem_t Data_mem;

extern char bufferRx[64];
extern char bufferTx[64];
extern bool flag_bufferRx;

/*---  Public Function Declaration  ------------------------------------------------------------ */

//! Limpia la memoria de datos
void ClearMemory(void);

//! Imprime los titlos de la tabla en consola
void PrintTableConsole(void);

//! Imprime los datos de la memoria en consola
void PrintConsoleMemory(void);

#endif
