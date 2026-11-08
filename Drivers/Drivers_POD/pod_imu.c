#include "pod_imu.h"
#include "pod_shared.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ── Periféricos externos de main.c ─────────────── */
extern MPU6050_Handle_t mpu;

/* ── IMU DEFINES ──────────────────────────── */
#define IMU_SAMPLE_MS            50U      // 20 Hz
#define IMU_TELEMETRY_MS         100U     // 10 Hz
#define MSG_LEN                  48U

#define ACCEL_SCALE_4G           8192.0f
#define GYRO_SCALE_250DPS        131.0f

#define GYRO_BIAS_SAMPLES        50U      // 2.5 s a 20 Hz
#define GYRO_CAL_MAX_DPS         10.0f    // reinicia calibración si hay movimiento
#define GYRO_MOTION_THRESH_DPS   3.0f
#define ACCEL_MOTION_THRESH_G    0.05f
#define MOTION_DEBOUNCE_MS       300U
#define AUTO_HOLD_MS             1000U
#define AUTO_OFF_MS              180000U  // 3 min

/* Banda de nivelación. En prototipo con montaje flexible se amplía;
 * ajustar cuando el sensor quede fijo en la carcasa definitiva. */
#define LEVEL_ON_DEG             2.0f
#define LEVEL_OFF_DEG            3.0f

#define ALPHA_MOVING             0.98f
#define ALPHA_STILL              0.95f

/* Ajustar estos signos si la respuesta del giro se opone al ángulo. */
#define GYRO_ROLL_SIGN           1.0f
#define GYRO_PITCH_SIGN          1.0f

/* ── Estado del filtro y calibración ─────── */
static float    fused_roll       = 0.0f;
static float    fused_pitch      = 0.0f;
static float    tare_roll        = 0.0f;
static float    tare_pitch       = 0.0f;
static float    gyro_bias_x      = 0.0f;
static float    gyro_bias_y      = 0.0f;
static float    gyro_bias_z      = 0.0f;
static float    bias_sum_x       = 0.0f;
static float    bias_sum_y       = 0.0f;
static float    bias_sum_z       = 0.0f;
static uint16_t bias_count       = 0;
static uint8_t  bias_ready       = 0;
static uint8_t  was_active       = 0;

static uint8_t  moving            = 0;
static uint8_t  hold_manual       = 0;
static uint8_t  hold_auto         = 0;
static uint8_t  auto_hold_armed   = 1;
static uint8_t  level_state       = 0;

static uint32_t imu_last_tick      = 0;
static uint32_t last_motion_tick   = 0;
static uint32_t last_telemetry_tick = 0;

/* ── Gesture (se conserva, no se usa en modo nivel) ── */
#define SHAKE_WINDOW     8
#define SHAKE_THRESHOLD  0.35f
#define SHAKE_COOLDOWN   1000U

static float    shake_buf[SHAKE_WINDOW];
static uint8_t  shake_idx     = 0;
static uint32_t shake_last    = 0;

static void IMU_ClearOutputs(void)
{
    IMU_Roll   = 0.0f;
    IMU_Pitch  = 0.0f;
    IMU_Level  = 0;
    IMU_Hold   = 0;
    IMU_Moving = 0;
}

static void IMU_ResetRuntime(void)
{
    fused_roll       = 0.0f;
    fused_pitch      = 0.0f;
    tare_roll        = 0.0f;
    tare_pitch       = 0.0f;

    gyro_bias_x      = 0.0f;
    gyro_bias_y      = 0.0f;
    gyro_bias_z      = 0.0f;
    bias_sum_x       = 0.0f;
    bias_sum_y       = 0.0f;
    bias_sum_z       = 0.0f;
    bias_count       = 0;
    bias_ready       = 0;

    moving           = 0;
    hold_manual      = 0;
    hold_auto        = 0;
    auto_hold_armed  = 1;
    level_state      = 0;

    imu_last_tick       = HAL_GetTick();
    last_motion_tick    = HAL_GetTick();
    last_telemetry_tick = 0;

    IMU_ClearOutputs();
}

void IMU_Tare(void)
{
    /* El ángulo absoluto es fused + tare. Al tarar, movemos el cero. */
    tare_roll  += fused_roll;
    tare_pitch += fused_pitch;

    fused_roll  = 0.0f;
    fused_pitch = 0.0f;

    hold_manual     = 0;
    hold_auto       = 0;
    auto_hold_armed = 0;
    level_state     = 0;

    IMU_Roll   = 0.0f;
    IMU_Pitch  = 0.0f;
    IMU_Level  = 0;
    IMU_Hold   = 0;

    last_motion_tick    = HAL_GetTick();
    last_telemetry_tick = 0;
}

void IMU_HoldToggle(void)
{
    hold_manual = hold_manual ? 0 : 1;

    if (hold_manual)
    {
        hold_auto       = 0;
        auto_hold_armed = 0;
    }

    IMU_Hold = (hold_manual || hold_auto) ? 1U : 0U;
}

void IMU_DetectGesture(float ax, float ay, float az)
{
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    shake_buf[shake_idx] = mag;
    shake_idx = (shake_idx + 1) % SHAKE_WINDOW;

    float mean = 0.0f;
    for (int i = 0; i < SHAKE_WINDOW; i++)
        mean += shake_buf[i];
    mean /= SHAKE_WINDOW;

    float var = 0.0f;
    for (int i = 0; i < SHAKE_WINDOW; i++)
        var += (shake_buf[i] - mean) * (shake_buf[i] - mean);
    var /= SHAKE_WINDOW;

    uint32_t now = HAL_GetTick();
    if (var > SHAKE_THRESHOLD && (now - shake_last) > SHAKE_COOLDOWN)
    {
        shake_last = now;
        POD_UART_Send("IMU:SHAKE\r\n");
    }
}

void IMU_Processloop(void)
{
    uint32_t now = HAL_GetTick();

    if (!imu_active)
    {
        if (was_active)
        {
            was_active = 0;
            IMU_ClearOutputs();
        }
        return;
    }

    if (!was_active)
    {
        IMU_ResetRuntime();
        was_active = 1;
        now = HAL_GetTick();
    }

    if ((now - imu_last_tick) < IMU_SAMPLE_MS)
        return;

    float dt = (now - imu_last_tick) / 1000.0f;
    imu_last_tick = now;

    if (dt <= 0.0f || dt > 0.25f)
        dt = IMU_SAMPLE_MS / 1000.0f;

    MPU6050_Accel_t accel;
    MPU6050_Gyro_t  gyro;

    if (MPU6050_GetMotion6(&mpu, &accel, &gyro) != MPU6050_OK)
        return;

    float ax_s = accel.ax / ACCEL_SCALE_4G;
    float ay_s = accel.ay / ACCEL_SCALE_4G;
    float az_s = accel.az / ACCEL_SCALE_4G;

    float gx_s = gyro.gx / GYRO_SCALE_250DPS;
    float gy_s = gyro.gy / GYRO_SCALE_250DPS;
    float gz_s = gyro.gz / GYRO_SCALE_250DPS;

    /* ── Remapeo por montaje vertical del MPU6050 ──────────────────
     * El sensor está soldado de pie: su eje +X queda vertical y apunta
     * hacia arriba cuando el dispositivo está nivelado.
     * Se rota el marco con una permutación cíclica (det = +1):
     *     Xv = +Y_s    Yv = +Z_s    Zv = +X_s
     * Se aplica igual al acelerómetro y al giroscopio para que el
     * filtro complementario reciba ejes coherentes.
     * ──────────────────────────────────────────────────────────── */
    float ax =  ay_s;
    float ay =  az_s;
    float az =  ax_s;

    float gx =  gy_s;
    float gy =  gz_s;
    float gz =  gx_s;

    float roll_acc  = atan2f(ay, az) * 180.0f / (float)M_PI - tare_roll;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI - tare_pitch;

    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);

    /* ── Calibración de bias del giroscopio ───────── */
    if (!bias_ready)
    {
        float gyro_raw_mag = sqrtf(gx*gx + gy*gy + gz*gz);

        if (gyro_raw_mag > GYRO_CAL_MAX_DPS ||
            fabsf(accel_mag - 1.0f) > 0.10f)
        {
            /* Si se mueve durante la calibración, reiniciarla. */
            bias_sum_x = 0.0f;
            bias_sum_y = 0.0f;
            bias_sum_z = 0.0f;
            bias_count = 0;

            last_motion_tick = now;
            moving = 1;
            IMU_Moving = 1;

            fused_roll  = roll_acc;
            fused_pitch = pitch_acc;
            IMU_Roll    = fused_roll;
            IMU_Pitch   = fused_pitch;
            return;
        }

        bias_sum_x += gx;
        bias_sum_y += gy;
        bias_sum_z += gz;
        bias_count++;

        if (bias_count >= GYRO_BIAS_SAMPLES)
        {
            gyro_bias_x = bias_sum_x / (float)bias_count;
            gyro_bias_y = bias_sum_y / (float)bias_count;
            gyro_bias_z = bias_sum_z / (float)bias_count;
            bias_ready = 1;
        }

        /* Mientras calibra, usar solo acelerómetro. */
        fused_roll  = roll_acc;
        fused_pitch = pitch_acc;
        IMU_Roll    = fused_roll;
        IMU_Pitch   = fused_pitch;
        IMU_Moving  = 0;
        return;
    }

    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;

    /* ── Detección de movimiento ──────────────────── */
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    uint8_t motion_now = (gyro_mag > GYRO_MOTION_THRESH_DPS) ||
                         (fabsf(accel_mag - 1.0f) > ACCEL_MOTION_THRESH_G);

    if (motion_now)
    {
        last_motion_tick = now;
        moving = 1;
        hold_auto = 0;
        auto_hold_armed = 1;
        last_activity_tick = now;
    }
    else if ((now - last_motion_tick) > MOTION_DEBOUNCE_MS)
    {
        moving = 0;
    }

    IMU_Moving = moving;

    /* ── Filtro complementario adaptativo ─────────── */
    float alpha = moving ? ALPHA_MOVING : ALPHA_STILL;

    fused_roll  = alpha * (fused_roll  + GYRO_ROLL_SIGN  * gx * dt)
                + (1.0f - alpha) * roll_acc;
    fused_pitch = alpha * (fused_pitch + GYRO_PITCH_SIGN * gy * dt)
                + (1.0f - alpha) * pitch_acc;

    /* ── Auto-hold ────────────────────────────────── */
    if (auto_hold_armed && !hold_manual && !moving &&
        (now - last_motion_tick) > AUTO_HOLD_MS)
    {
        hold_auto = 1;
        auto_hold_armed = 0;
    }

    if (moving)
        hold_auto = 0;

    IMU_Hold = (hold_manual || hold_auto) ? 1U : 0U;

    if (!IMU_Hold)
    {
        IMU_Roll  = fused_roll;
        IMU_Pitch = fused_pitch;
    }

    /* ── Estado LEVEL con histéresis ────────────────
     * Solo depende del ángulo: no exige que el detector de
     * movimiento esté en reposo, para que responda mientras se
     * ajusta el nivel. */
    float abs_roll  = fabsf(IMU_Roll);
    float abs_pitch = fabsf(IMU_Pitch);

    if (level_state)
    {
        if (abs_roll > LEVEL_OFF_DEG || abs_pitch > LEVEL_OFF_DEG)
            level_state = 0;
    }
    else
    {
        if (abs_roll <= LEVEL_ON_DEG && abs_pitch <= LEVEL_ON_DEG)
            level_state = 1;
    }

    IMU_Level = level_state;

    /* ── Telemetría ───────────────────────────────── */
    if ((now - last_telemetry_tick) >= IMU_TELEMETRY_MS)
    {
        last_telemetry_tick = now;

        int16_t roll_i  = (int16_t)(IMU_Roll  * 10.0f);
        int16_t pitch_i = (int16_t)(IMU_Pitch * 10.0f);
        char msg[MSG_LEN];

        snprintf(msg, MSG_LEN, "IMU:L%d,R%d,P%d\r\n",
                 (int)IMU_Level, roll_i, pitch_i);
        POD_UART_Send(msg);
    }

    /* ── Auto-off por inmovilidad ─────────────────── */
    if (!moving && (now - last_motion_tick) >= AUTO_OFF_MS)
    {
        imu_active = 0;
        was_active = 0;

        hold_manual     = 0;
        hold_auto       = 0;
        auto_hold_armed = 0;
        level_state     = 0;

        IMU_ClearOutputs();
        last_activity_tick = now;

        POD_UART_Send("IMU:OFF\r\n");
    }
}
