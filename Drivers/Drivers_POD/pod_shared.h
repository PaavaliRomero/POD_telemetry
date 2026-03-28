
// pod_shared.h
#ifndef POD_SHARED_H
#define POD_SHARED_H

#include <stdint.h>

/**
 * @brief Estados de la máquina de estados principal del POD.
 */
typedef enum {
    POD_STATE_INIT,   /**< Inicialización del sistema. */
    POD_STATE_IDLE,   /**< En espera de comandos. */
    POD_STATE_ERROR   /**< Error de comunicación. */
} pod_state_enum;

/**
 * @brief Estados de la interfaz gráfica OLED.
 */
typedef enum {
    POD_STATE_AnimaLoad,    /**< Animación de carga inicial. */
    POD_STATE_MainScreen,   /**< Pantalla principal con datos. */
    POD_STATE_FaultScreen   /**< Pantalla de error. */
} UI_state_enum;

/** @brief Voltaje de batería en mV. Actualizado por POD_FSM_BATT_func. */
extern volatile uint16_t      VBATT;

/** @brief Estado actual del LED. 1 = encendido, 0 = apagado. */
extern volatile uint8_t       State_LED;

/** @brief Estado actual de la interfaz gráfica. */
extern volatile UI_state_enum pod_state_ui;

/** @brief Estado actual de la FSM principal. */
extern volatile pod_state_enum pod_state;

/** @brief Flag de activación del IMU. 1 = activo, 0 = inactivo. */
extern volatile uint8_t        imu_active;

/** @brief Último tick de actividad para el cálculo del timeout de sleep. */
extern uint32_t                last_activity_tick;

#endif
