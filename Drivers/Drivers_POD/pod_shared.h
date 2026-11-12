
// pod_shared.h
#ifndef POD_SHARED_H
#define POD_SHARED_H

#include <stdint.h>

/** @brief Tamaño máximo del buffer de comandos recibidos por UART. */
#define POD_CMD_BUF_SIZE 64U

/**
 * @brief Compilar y ejecutar la interfaz OLED.
 *
 * 0 = sin pantalla (por defecto): no se inicializa la OLED ni se refresca.
 *     El codigo de ui_oled.c y ssd1306.c se sigue compilando para detectar
 *     roturas, pero el linker lo descarta porque nadie lo referencia.
 * 1 = con pantalla: se inicializa, se dibuja la pantalla de nivel y se
 *     apaga/enciende junto con el modo sleep.
 *
 * Motivacion: la OLED comparte I2C2 con el MPU6050 y ssd1306.c usa
 * HAL_MAX_DELAY en todas sus escrituras. Un panel ausente o colgado puede
 * bloquear el bus del sensor, y cada refresco mueve ~0.5 KB por I2C
 * (~12 ms a 400 kHz) cada 100 ms mientras el IMU esta activo.
 * Ver docs/OLED_BUILD.md para el analisis completo.
 */
#ifndef POD_USE_OLED
#define POD_USE_OLED 0
#endif

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

/** @brief Ángulo roll filtrado y con tare aplicado, en grados. */
extern volatile float          IMU_Roll;

/** @brief Ángulo pitch filtrado y con tare aplicado, en grados. */
extern volatile float          IMU_Pitch;

/** @brief 1 si el nivel está dentro del umbral de nivelación; 0 si no. */
extern volatile uint8_t        IMU_Level;

/** @brief 1 si la lectura está congelada (hold manual o automático). */
extern volatile uint8_t        IMU_Hold;

/** @brief 1 si el giroscopio detecta movimiento; 0 si está quieto. */
extern volatile uint8_t        IMU_Moving;

/** @brief Último tick de actividad para el cálculo del timeout de sleep. */
extern uint32_t                last_activity_tick;

/**
 * @brief Envía un mensaje por UART1 al ESP32.
 * @param msg Cadena terminada en '\0'.
 * @return 0 si se transmitió, -1 si falló.
 */
int POD_UART_Send(const char *msg);

/**
 * @brief Copia de forma atómica el último comando recibido por UART.
 * @param dst Buffer destino.
 * @param max_len Tamaño del buffer destino.
 * @return 1 si había un comando, 0 si no.
 */
uint8_t POD_UART_TakeCommand(char *dst, uint16_t max_len);

/**
 * @brief Indica si hay comandos completos pendientes en la cola UART.
 * @return 1 si hay al menos un comando pendiente, 0 si no.
 */
uint8_t POD_UART_HasPendingCommand(void);

/**
 * @brief Rearma la recepción UART después de salir de STOP.
 */
void POD_UART_Resume(void);

#endif
