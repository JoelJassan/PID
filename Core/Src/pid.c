/**
 * @file pid.c
 * @author Joel Jassan <joeljassan@hotmail.com>
 * @brief  PID embebido en un STM32F4
 * @date 2025-04
 *
 * @copyright Copyright (c) 2025. All rights reserved.
 *
 */
/* --------------------------------------------------------------------------------------------- */

/*---  Includes  ------------------------------------------------------------------------------- */
#include "pid.h"

/*---  Macros and Definitions  ----------------------------------------------------------------- */

/*---  Private Data Declaration  --------------------------------------------------------------- */

/*---  Public Data Declaration  ---------------------------------------------------------------- */

/*---  Private Function Declaration  ----------------------------------------------------------- */

/*---  Public Function Declaration  ------------------------------------------------------------ */

/*---  Private Data Definition  ---------------------------------------------------------------- */

/*---  Public Data Definition  ----------------------------------------------------------------- */

PID_controller_t PID;

/*---  Private Function Definition  ------------------------------------------------------------ */

/*---  Public Function Definition  ------------------------------------------------------------- */

/*---  Private Function Implementation  -------------------------------------------------------- */

/**
 * @brief Inicializa los parámetros del PID
 * 
 * @param Kp Constante proporcional, en el dominio S
 * @param Ki Constante integrativa, en el dominio S
 * @param Kd Constante derivativa, en el dominio S
 * @param Ts tiempo de muestreo, en [s]
 */
void PIDInit(float Kp, float Ki, float Kd, float Ts) {
    
    // Inicializa parámetros propios del PID
    PID.Kp_z = Kp;
    PID.Ki_z = Ki * Ts;
    PID.Kd_z = Kd / Ts;
    PID.Ts = Ts;
}

/**
 * @brief Reinicia las variables matemáticas del PID
 * 
 */
void PIDReset() {

    //Inicializa las variables matemáticas del PID
    PID.Maths.error = 0.0f;
    PID.Maths.set_point = 0.0f;
    PID.Maths.integral = 0.0f;
    PID.Maths.derivada = 0.0f;
    PID.Maths.error_prev = 0.0f;
    PID.Maths.integral_prev = 0.0f;
}

/**
 * @brief Ajusta el set point del PID
 * 
 * @param set_point Valor del set point
 * @details El set point se limita entre 0 y 5.8V, que es el rango de tensión de salida del tacometro
 */
void SetSetPoint(float set_point) {
    if (set_point >= 5.8) PID.Maths.set_point = 5.8;
    else if (set_point <= 0) PID.Maths.set_point = 0;
    else PID.Maths.set_point = set_point;
}

/**
 * @brief Ajusta el PWM del controlador
 * 
 * @param PWM_percent Valor del PWM en porcentaje
 * @details El PWM se limita entre 0 y 100%
 */
void AdjustPWM(uint8_t PWM_percent) {
    // Ajusta el PWM del controlador
    if (PWM_percent >= 100) PID.In_Out.PWM_percent = 100;
    else if (PWM_percent <= 0) PID.In_Out.PWM_percent = 0;
    else PID.In_Out.PWM_percent = PWM_percent;
}

/**
 * @brief Actualiza el PID
 * 
 * @param ADC_to_V Valor de tensión medido por el ADC
 * @return uint8_t PWM_percent (salida de tensión %)
 * @details El PID se actualiza cada vez que se recibe un nuevo valor de tensión medido por el ADC.
 */
uint8_t PIDRefresh(float ADC_to_V){
    PID.In_Out.ADC_to_V = ADC_to_V;

    PID.Maths.error = PID.Maths.set_point - PID.In_Out.ADC_to_V;
    PID.Maths.integral = PID.Ki_z*PID.Maths.error + PID.Maths.integral_prev;
    PID.Maths.derivada = PID.Kd_z*(PID.Maths.error - PID.Maths.error_prev);

    //Adaptación: tension a pwm
    PID.In_Out.PWM_percent = PID.Kp_z*PID.Maths.error + PID.Maths.integral + PID.Maths.derivada;
    AdjustPWM(PID.In_Out.PWM_percent);

    //memoria para próximo ciclo
    PID.Maths.error_prev = PID.Maths.error;
    PID.Maths.integral_prev = PID.Maths.integral;

    return PID.In_Out.PWM_percent;
}

/**
 * @brief Crea la estructura del PID e inicializa los parámetros
 * 
 * @param Kp Constante proporcional, en el dominio S
 * @param Ki Constante integrativa, en el dominio S
 * @param Kd Constante derivativa, en el dominio S
 * @param Ts tiempo de muestreo, en [s]
 * * @details Inicializa el PID y lo pone en cero. Se debe llamar al menos una vez antes de usar el PID.
 * @note Se recomienda llamar a esta función en el main.c antes de entrar al bucle infinito.
 */
void PIDCreate(float Kp, float Ki, float Kd, float Ts) {
    PIDInit(Kp, Ki, Kd, Ts);
    PIDReset();
}

/*---  Public Function Implementation  --------------------------------------------------------- */

/*---  End of File  ---------------------------------------------------------------------------- */
