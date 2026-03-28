#ifndef POD_FSM_H
#define POD_FSM_H

#include "stm32f1xx_hal.h"
#include "pod_shared.h"

/**
 * @brief Ejecuta un ciclo de la máquina de estados principal del POD.
 * @note  Debe llamarse en el loop principal de main.c.
 */
void FINITE_STATE_MACHINE_MAIN(void);

/**
 * @brief Ejecuta un ciclo de la interfaz gráfica OLED.
 * @note  Debe llamarse en el loop principal de main.c.
 */
void POD_UI_TASK(void);

/**
 * @brief Solicita y transmite el nivel de batería al ESP32.
 */
void POD_FSM_BATT_func(void);

/**
 * @brief Realiza toggle del LED y notifica estado al ESP32.
 */
void POD_FSM_LED_func(void);

/**
 * @brief Responde al comando PING del ESP32.
 */
void POD_FSM_PING_func(void);

/**
 * @brief Activa o desactiva el IMU y notifica al ESP32.
 */
void POD_FSM_MPU_Toggle_func(void);

/**
 * @brief Pone el STM32 en modo sleep y gestiona el despertar.
 * @note  Al despertar reinicia el reloj y reactiva periféricos.
 */
void POD_ModeSleep_Func(void);

#endif
