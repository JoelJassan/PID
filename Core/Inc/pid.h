/**
 * @file pid.h
 * @author Joel Jassan <joeljassan@hotmail.com>
 * @brief  Declaracion de variables para un PID embebido en un STM32F4
 * @date 2025-04
 *
 * @copyright Copyright (c) 2025. All rights reserved.
 *
 */
/* --------------------------------------------------------------------------------------------- */

#ifndef PID_H
#define PID_H

/*---  Includes  ------------------------------------------------------------------------------- */
#include <stdint.h>

/*---  Macros  --------------------------------------------------------------------------------- */

/*---  Definitions  ---------------------------------------------------------------------------- */

/*---  Public Data Declaration  ---------------------------------------------------------------- */

/*
 * @brief Estructura con los parámetros matemáticos de un controlador PID
 */
typedef struct PID_Maths_s{
    float error;         /**< Error actual (set_point - valor medido) */
    float set_point;     /**< Set point del PID (valor deseado) */
    float integral;      /**< Integral del error */
    float derivada;      /**< Derivada del error */
    float error_prev;    /**< Error anterior (error[n-1]) */
    float integral_prev; /**< Integral anterior (integral[n-1]) */
} PID_Maths_t;

/*
 * @brief Estructura con los parámetros de entrada salida de un contrlador PID
 */
typedef struct In_Out_s{
    float ADC_to_V;      /**< Valor de tensión medido por el ADC */
    uint8_t PWM_percent; /**< Valor de PWM calculado por el PID (salida de tensión %) */
} In_Out_t;

/*
 * @brief Estructura con los parámetros un controlador PID  
 */
typedef struct PID_controller_s{
    float Ts;            /**< Tiempo de muestreo (= 1/fs) */
    float Kp_z;          /**< Ganancia proporcional en el dominio Z */
    float Ki_z;          /**< Ganancia integral en el dominio Z */
    float Kd_z;          /**< Ganancia derivativa en el dominio Z */
    PID_Maths_t Maths;
    In_Out_t In_Out;     /**< Estructura con los parámetros de entrada y salida del PID */
} PID_controller_t;

extern PID_controller_t PID;

/*---  Public Function Declaration  ------------------------------------------------------------ */
//! Inicializa los parámetros del PID
void PIDInit(float Kp, float Ki, float Kd, float Ts);

//! Pone en cero de las variables matemáticas del PID
void PIDReset(void);

//! Actualiza el PID
uint8_t PIDRefresh(float ADC_to_V);

//! Crea la estructura del PID e inicializa los parámetros
void PIDCreate(float Kp, float Ki, float Kd, float Ts);

//! Ajusta el set point del PID
void SetSetPoint(float set_point);

void AdjustPWM(uint8_t PWM_percent);

#endif
