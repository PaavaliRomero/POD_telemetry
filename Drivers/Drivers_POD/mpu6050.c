
#include "mpu6050.h"

#define MPU6050_timeOUT 100

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *dev,I2C_HandleTypeDef *hi2c, uint8_t address)
{

	/* Inicialización básica del handle */

    if (dev == NULL || hi2c == NULL)
        return MPU6050_ERROR;

    dev->hi2c = hi2c;
    dev->address = address;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *dev, uint8_t data, uint8_t reg)
{
	if(dev == NULL)
		return MPU6050_ERROR;

	if(HAL_I2C_Mem_Write(dev->hi2c,
					  dev->address,
					  reg,
					  I2C_MEMADD_SIZE_8BIT,
					  &data,
					  1,
					  MPU6050_timeOUT)!= HAL_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ConfigModifi(MPU6050_Handle_t *dev)
{
	if(dev == NULL)
		return MPU6050_ERROR;

	/*Wake up + PLL*/
	MPU6050_WriteReg(dev, 0x01, MPU6050_REG_PWR_MGMT1);
	/*Sample Rate = 1kHz*/
	MPU6050_WriteReg(dev, 0x00, MPU6050_REG_SMPLRT_DIV);
	/* DLPF = 21 Hz (accel) / 20 Hz (gyro).
	 * El lazo muestrea a 20 Hz; 42 Hz dejaba pasar vibración que
	 * luego aparecía como aliasing. Para más suavizado usar 0x05
	 * (10 Hz); para más respuesta usar 0x03 (42 Hz). */
	MPU6050_WriteReg(dev, 0x04, MPU6050_REG_CONFIG_DLPF);
	/*Gyro +- 250 dps*/
	MPU6050_WriteReg(dev, MPU6050_Gyro_FS_250, MPU6050_REG_GYRO_CONFIG);
	dev->gyro_config = 250;
	/*Accel +-4g*/
	MPU6050_WriteReg(dev, MPU6050_Accel_FS_4G, MPU6050_REG_ACCEL_CONFIG);
	dev->accel_config = 4;

	return MPU6050_OK;

}

MPU6050_Status_t MPU6050_TestConnection(MPU6050_Handle_t *dev)
{
	/* Verifica que el dispositivo responda en el bus */
    if (HAL_I2C_IsDeviceReady(dev->hi2c,
                              dev->address,
                              3,
                              MPU6050_timeOUT) == HAL_OK)
    {
        return MPU6050_OK;
    }

    return MPU6050_NOT_FOUND;
}


MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Handle_t *dev,uint8_t *value)
{
	/* Lee el registro WHO_AM_I */

    if (HAL_I2C_Mem_Read(dev->hi2c,
                         dev->address,
                         MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT,
                         value,
                         1,
                         MPU6050_timeOUT) == HAL_OK)
    {
        return MPU6050_OK;
    }

    return MPU6050_ERROR;
}

MPU6050_Status_t MPU6050_GetGyroXYZ(MPU6050_Handle_t *dev, MPU6050_Gyro_t *gyro)
{
	/* That function read all registers, begin at register 0x43 and go up to register
	 *  0x48. Consequently read all follow registers:
	 *
	 *	0x43 --> GYRO_XOUT_H
	 *	0x44 --> GYRO_XOUT_L
	 *	0x45 --> GYRO_YOUT_H
	 *	0x46 --> GYRO_YOUT_L
	 *	0x47 --> GYRO_ZOUT_H
	 *	0x48 --> GYRO_ZOUT_L
	 *
	 *	And save all values on a typedef struct 'MPU6050_Gyro_t'. No necesary print only accel by
	 *	axis
	 * */
	uint8_t raw[6];

	if(dev->hi2c == NULL || gyro == NULL)
		return HAL_ERROR;

	if (HAL_I2C_Mem_Read(dev->hi2c,
		                 MPU6050_I2C_ADDR,
		                 MPU6050_REG_GYRO_XOUT_H,
		                 I2C_MEMADD_SIZE_8BIT,
		                 raw,
		                 6,
		                 MPU6050_timeOUT) == HAL_OK)
	{
        gyro->gx = (int16_t)((raw[0] << 8) | raw[1]);
        gyro->gy = (int16_t)((raw[2] << 8) | raw[3]);
        gyro->gz = (int16_t)((raw[4] << 8) | raw[5]);

        return MPU6050_OK;
	}
	else
	{
		return MPU6050_ERROR;
	}
}

MPU6050_Status_t MPU6050_GetAccelXYZ(MPU6050_Handle_t *dev, MPU6050_Accel_t *accel)
{
	/* That function read all registers, begin at register 0x3B and go up to register
	 *  0x40. Consequently read all follow registers:
	 *
	 *	0x3B --> ACCEL_XOUT_H
	 *	0x3C --> ACCEL_XOUT_L
	 *	0x3D --> ACCEL_YOUT_H
	 *	0x3E --> ACCEL_YOUT_L
	 *	0x3F --> ACCEL_ZOUT_H
	 *	0x40 --> ACCEL_ZOUT_L
	 *
	 *	And save all values on a typedef struct 'MPU6050_Accel_t'. No necesary print only accel by
	 *	axis
	 * */

	uint8_t raw[6];

	if(dev->hi2c == NULL || accel == NULL)
		return HAL_ERROR;

	if (HAL_I2C_Mem_Read(dev->hi2c,
		                 MPU6050_I2C_ADDR,
		                 MPU6050_REG_ACCEL_XOUT_H,
		                 I2C_MEMADD_SIZE_8BIT,
		                 raw,
		                 6,
		                 MPU6050_timeOUT) == HAL_OK)
	{
        accel->ax = (int16_t)((raw[0] << 8) | raw[1]);
        accel->ay = (int16_t)((raw[2] << 8) | raw[3]);
        accel->az = (int16_t)((raw[4] << 8) | raw[5]);

        return MPU6050_OK;
	}
	else
	{
		return MPU6050_ERROR;
	}
}

MPU6050_Status_t MPU6050_GetMotion6(MPU6050_Handle_t *dev,
                                    MPU6050_Accel_t *accel,
                                    MPU6050_Gyro_t *gyro)
{
	uint8_t raw[14];

	if (dev == NULL || dev->hi2c == NULL || accel == NULL || gyro == NULL)
		return MPU6050_ERROR;

	/* 0x3B..0x40: accel, 0x41..0x42: temp, 0x43..0x48: gyro */
	if (HAL_I2C_Mem_Read(dev->hi2c,
	                     dev->address,
	                     MPU6050_REG_ACCEL_XOUT_H,
	                     I2C_MEMADD_SIZE_8BIT,
	                     raw,
	                     14,
	                     MPU6050_timeOUT) != HAL_OK)
	{
		return MPU6050_ERROR;
	}

	accel->ax = (int16_t)((raw[0]  << 8) | raw[1]);
	accel->ay = (int16_t)((raw[2]  << 8) | raw[3]);
	accel->az = (int16_t)((raw[4]  << 8) | raw[5]);

	gyro->gx  = (int16_t)((raw[8]  << 8) | raw[9]);
	gyro->gy  = (int16_t)((raw[10] << 8) | raw[11]);
	gyro->gz  = (int16_t)((raw[12] << 8) | raw[13]);

	return MPU6050_OK;
}
