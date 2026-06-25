
#include "pod_imu.h"
#include "pod_shared.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ── Periféricos externos de main.c ─────────────── */
extern UART_HandleTypeDef huart1;
extern MPU6050_Handle_t   mpu;

/* ── IMU DEFINES ──────────────────────────── */
#define IMU_SAMPLE_MS    50U     // leer cada 50ms (20Hz)
#define IMU_ZONE_THRESH  45.0f   // grados para tilt lateral (roll)
#define IMU_PITCH_THRESH 30.0f   // grados para tilt adelante/atrás (pitch)
#define SHAKE_WINDOW 	 8 		//guarda las ultimas 8 muestras
#define SHAKE_THRESHOLD  0.35f   // varianza mínima para considerar shake
#define SHAKE_COOLDOWN   1000U   // ms entre un shake y el siguiente
#define MSG_LEN 48U //Tamaño del mensjae a enviar a ESP32
#define ACCEL_SCALE_4G 8192.0f

/* Flag: el ESP32 activó el IMU con STM:MPU */
static 	 IMU_Zone imu_zone     = ZONE_FLAT;
static   IMU_Zone imu_zone_prev= ZONE_FLAT;
static   uint32_t imu_last_tick = 0;


/* Function DetectGesture */
static float    shake_buf[SHAKE_WINDOW];
static uint8_t  shake_idx     = 0;
static uint32_t shake_last    = 0;

void IMU_DetectGesture(float ax, float ay, float az)
{
    /* Magnitud total de la aceleración */
    float mag = sqrtf(ax*ax + ay*ay + az*az);


    /* Guardar en ventana circular */
    shake_buf[shake_idx] = mag;
    shake_idx = (shake_idx + 1) % SHAKE_WINDOW;


    /* Calcular varianza de la ventana */
    float mean = 0;
    for (int i = 0; i < SHAKE_WINDOW; i++) mean += shake_buf[i];
    mean /= SHAKE_WINDOW;

    float var = 0;
    for (int i = 0; i < SHAKE_WINDOW; i++)
        var += (shake_buf[i] - mean) * (shake_buf[i] - mean);
    var /= SHAKE_WINDOW;


    /* Shake detectado si varianza supera umbral */
    uint32_t now = HAL_GetTick();
    if (var > SHAKE_THRESHOLD &&
        (now - shake_last) > SHAKE_COOLDOWN)
    {
        shake_last = now;
        HAL_UART_Transmit(&huart1,
            (uint8_t*)"IMU:SHAKE\r\n", 11, 100);
    }

}

void IMU_Processloop(void)
{
    if (!imu_active) return;
    if ((HAL_GetTick() - imu_last_tick) < IMU_SAMPLE_MS) return;
    imu_last_tick = HAL_GetTick();

	char msg[MSG_LEN];
	MPU6050_Accel_t accel;

	if (MPU6050_TestConnection(&mpu) != MPU6050_OK)
	{
		/*Check conection between devices*/

		sprintf(msg, "Device not ready\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	if (MPU6050_GetAccelXYZ(&mpu, &accel) != MPU6050_OK) return;

	float ax_raw = accel.ax / ACCEL_SCALE_4G;
	float ay_raw = accel.ay / ACCEL_SCALE_4G;
	float az_raw = accel.az / ACCEL_SCALE_4G;

	float roll	= atan2f(ay_raw, az_raw) * 180.0f / M_PI;
	float pitch	= atan2f(-ax_raw, sqrtf(ay_raw*ay_raw + az_raw*az_raw)) * 180.0f / M_PI;

    if      (az_raw < -0.8f)             imu_zone = ZONE_FACEDOWN;
    else if (roll  >  IMU_ZONE_THRESH)   imu_zone = ZONE_RIGHT;
    else if (roll  < -IMU_ZONE_THRESH)   imu_zone = ZONE_LEFT;
    else if (pitch >  IMU_PITCH_THRESH)  imu_zone = ZONE_FORWARD;
    else if (pitch < -IMU_PITCH_THRESH)  imu_zone = ZONE_BACK;
    else imu_zone = ZONE_FLAT;

    int16_t roll_i  = (int16_t)(roll  * 10.0f);
    int16_t pitch_i = (int16_t)(pitch * 10.0f);

    /* --- Transmisión continua: zona + valores numéricos --- */
    snprintf(msg, MSG_LEN, "IMU:Z%d,R%d,P%d\r\n", (int)imu_zone, roll_i, pitch_i);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    /* Solo transmitir si cambió la zona */
    if (imu_zone != imu_zone_prev) {
        imu_zone_prev = imu_zone;
    }

    /*This function detect gestures*/
    IMU_DetectGesture(ax_raw, ay_raw, az_raw);

    // Detección de movimiento por software
    float mag = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
    if (fabsf(mag - 1.0f) > 0.3f)
        last_activity_tick = HAL_GetTick();
}
