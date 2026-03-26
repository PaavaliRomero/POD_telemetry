
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Dirección por defecto (AD0 = GND) */
#define MPU6050_I2C_ADDR      (0x68 << 1)		//** b01101000 -> d208 */

/* Registros mínimos */
#define MPU6050_REG_ACCEL_XOUT_H 	0x3B
#define MPU6050_REG_GYRO_XOUT_H 	0x43
#define MPU6050_REG_WHO_AM_I  		0x75
#define MPU6050_REG_PWR_MGMT1 		0x6B
#define MPU6050_REG_SMPLRT_DIV		0x19
#define MPU6050_REG_CONFIG_DLPF		0x1A
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C

/*BITS states by function GYRO_CONFIG (fullscale)*/
#define MPU6050_Gyro_FS_250 		(0 << 3)	//** b00000 -> 0	*/
#define MPU6050_Gyro_FS_500 		(1 << 3)	//** b01000 -> d8	*/
#define MPU6050_Gyro_FS_1k 			(2 << 3)	//** b10000 -> d16	*/
#define MPU6050_Gyro_FS_2k 			(3 << 3)	//** b11000 -> d24	*/

/*BITS states by funcion Accel_Config (fullscale)*/
#define MPU6050_Accel_FS_2G			(0 << 3)	//** b00000 -> 0	*/
#define MPU6050_Accel_FS_4G 		(1 << 3)	//** b01000 -> d8	*/
#define MPU6050_Accel_FS_8G			(2 << 3)	//** b10000 -> d16	*/
#define MPU6050_Accel_FS_16G		(3 << 3)	//** b11000 -> d24	*/

/**
 * @brief Estados para las funciones de la libreria.
 *
 *  */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR,
    MPU6050_NOT_FOUND
} MPU6050_Status_t;

/**
 * @brief Estructura para guardado config del sensor.
 *
 * */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;

	uint16_t accel_config;
	uint16_t  gyro_config;

} MPU6050_Handle_t;

/**
 * @brief Estructura para almacenado de variables XYZ accelerometro.
 *
 * */
typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
} MPU6050_Accel_t;

/**
 * @brief Estructura para almacenado de variables XYZ giroscopio.
 *
 * */
typedef struct {
	int16_t gx;
	int16_t gy;
	int16_t gz;
} MPU6050_Gyro_t;

/**
 * @brief Inicializa la escritura y el callback del sensor.
 *
 * @param *dev: Es el manejador del sensor.
 * @param *hi2c: Manejador de la comunicacion I2c.
 * @param address: Direccion del sensor.
 *
 * @return: MPU6050_OK si no hubieron problemas con el registro de las entradas.
 * @return: MPU6050_FAil si no se escribieron correctamente las entradas.
 *
 * @note: Debe llamarse una sola vez para evitar problemas de comunicacion.
 *
 * @see: MPU6050_Handle_t, I2C_HandleTypeDef y MPU6050_I2C_ADDR.
 * */

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *dev,I2C_HandleTypeDef *hi2c,uint8_t address);

/**
 * @brief Sirve para obtener las mediciones del giroscopio crudas del sensor.
 *
 * @param *dev: Es el manejador del sensor.
 * @param *gyro: Estructura para XYZ.
 *
 * @return: MPU6050_OK si pudo obtener las mediciones.
 * @return: MPU6050_FAIL si no pudo obtner las mediciones.
 *
 * @note: Esta funcion solo lee datos no estan convertidos con ningun filtro aun.
 *
 * */
MPU6050_Status_t MPU6050_GetGyroXYZ(MPU6050_Handle_t *dev, MPU6050_Gyro_t *gyro);

/**
 * @brief Sirve para obtener las mediciones del acelerometro crudas del sensor.
 *
 * @param *dev: Es el manejador del sensor.
 * @param *accel: Estructura para XYZ.
 *
 * @return: MPU6050_OK si pudo obtener las mediciones.
 * @return: MPU6050_FAIL si no pudo obtner las mediciones.
 *
 * @note: Esta funcion solo lee datos no estan convertidos con ningun filtro aun.
 *
 * */
MPU6050_Status_t MPU6050_GetAccelXYZ(MPU6050_Handle_t *dev, MPU6050_Accel_t *accel);

/**
 * @brief verifica si el sensor responde correctamente.
 *
 * @param *dev: Es el manejador del sensor.
 *
 * @return: MPU6050_OK si pudo realizar el test.
 * @return: MPU6050_FAIL si no pudo realizar el test.
 *
 * */
MPU6050_Status_t MPU6050_TestConnection(MPU6050_Handle_t *dev);

/**
 * @brief Sirve para configurar los parametros del sensor.
 *
 * @param *dev: Es el manejador del sensor.
 *
 * @return: MPU6050_OK si se escribieron todas las configuraciones.
 * @return: MPU6050_FAIL si fallo en la escritura.
 *
 * @note: Esta funcion solo escribe registros por lo que se puede usar
 * 		  para registrar varias veces si se entra en un modo sleep del
 * 		  sensor.
 *
 * */
MPU6050_Status_t MPU6050_ConfigModifi(MPU6050_Handle_t *dev);

/**
 * @brief Sirve para obtener el resitro por defecto de whoiam del sensor.
 *
 * @param *dev: Es el manejador del sensor.
 * @param *value: el registro para whoiam.
 *
 * @return: MPU6050_OK si pudo obtener la direccion.
 * @return: MPU6050_FAIL si no pudo obtner la direccion.
 *
 * @note: Esta funcion solo lee la direccion del sensor.
 *
 * */
MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Handle_t *dev,uint8_t *value);


#endif
