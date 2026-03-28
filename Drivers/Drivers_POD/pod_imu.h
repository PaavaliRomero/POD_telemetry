
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

#endif
