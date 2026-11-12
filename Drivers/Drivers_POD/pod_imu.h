
#ifndef POD_IMU_H
#define POD_IMU_H

#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include <stdint.h>

/**
 *@brief Zonas de orientacion para la funcion IMU
 * */
typedef enum {
    ZONE_FLAT     = 0, /** Superficie plana */
    ZONE_RIGHT    = 1, /** Inclinado hacia derecha */
    ZONE_LEFT     = 2, /** Inclinado hacia la izquierda */
    ZONE_FORWARD  = 3, /** Inclinado hacia adelante */
    ZONE_BACK     = 4, /** Inclinacion hacia atras */
    ZONE_FACEDOWN = 5, /** Se encuientra boca abajo */
} IMU_Zone;

/**
 * @brief Gestos detectados por el IMU
 * */
typedef enum {
    GESTURE_NONE  = 0, /**< Sin gesto detectado. */
    GESTURE_SHAKE = 1, /**< Gesto de agitación detectado. */
    GESTURE_FLIP  = 2, /**< Gesto de volteo detectado. */
} IMU_Gesture;

/**
 * @brief Ejecuta un ciclo de lectura y procesamiento del IMU.
 * @note  Debe llamarse en el loop principal de main.c.
 * @note  Solo procesa si imu_active está habilitado.
 */
void IMU_Processloop(void);

/**
 * @brief Detecta gestos basándose en la aceleración del sensor.
 *
 * Usa una ventana circular para calcular la varianza de la
 * magnitud de aceleración y detectar el gesto de shake.
 *
 * @param ax Aceleración normalizada en eje X.
 * @param ay Aceleración normalizada en eje Y.
 * @param az Aceleración normalizada en eje Z.
 */
void IMU_DetectGesture(float ax, float ay, float az);

/**
 * @brief Fija la inclinación actual como referencia cero.
 * @note  Reinicia el hold y la referencia de movimiento.
 * @note  El cero se persiste en flash y se recupera al reactivar el IMU.
 */
void IMU_Tare(void);

/**
 * @brief Borra el cero guardado y vuelve al angulo absoluto del sensor.
 *
 * Convierte el valor relativo actual a absoluto y despues borra la pagina
 * de flash, de modo que no hay salto en la lectura. Es idempotente: si no
 * habia nada guardado, responde igual y no cambia el comportamiento.
 *
 * @return 1 si la pagina quedo borrada, 0 si fallo la escritura en flash.
 */
uint8_t IMU_TareClear(void);

/**
 * @brief Alterna el hold manual de la lectura.
 * @note  El hold manual tiene prioridad sobre el auto-hold.
 */
void IMU_HoldToggle(void);

/**
 * @brief Alterna el modo LIVE (desactiva el auto-hold indefinidamente).
 *
 * Con el auto-hold la telemetria repite el ultimo valor congelado, lo que
 * impide medir el ruido real. En LIVE la salida se actualiza siempre.
 * No se pierde al ejecutar TARE; si se pierde al reiniciar el IMU o en
 * el auto-off.
 */
void IMU_LiveToggle(void);

/**
 * @brief Devuelve 1 si el modo LIVE esta activo, 0 si no.
 */
uint8_t IMU_LiveState(void);

#endif
