#include "pod_imu.h"
#include "pod_shared.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ── Periféricos externos de main.c ─────────────── */
extern MPU6050_Handle_t mpu;
extern I2C_HandleTypeDef hi2c2;

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

/* ── Robustez contra golpes, vibración y ruido ──────────────────
 * Un golpe o una caída producen muestras fuera de rango. Se
 * descartan antes de tocar el filtro. La magnitud de gravedad en
 * reposo es ~1 g; se acepta solo una ventana razonable. */
#define ACCEL_VALID_MIN_G        0.60f
#define ACCEL_VALID_MAX_G        1.40f

/* Debounce del estado LEVEL: un golpe breve no debe apagar el LED. */
#define LEVEL_ON_DEBOUNCE_MS     200U
#define LEVEL_OFF_DEBOUNCE_MS    400U

/* Tras varios fallos seguidos del bus I2C, intentar recuperarlo. */
#define IMU_I2C_FAIL_LIMIT       5U

/* No reintentar la recuperacion mas de una vez por segundo: si el bus
 * esta realmente muerto, martillear el periferico no ayuda. */
#define IMU_I2C_RECOVER_COOLDOWN_MS 1000U

/* Bus clear: hasta 9 pulsos de SCL para que el esclavo termine la
 * transaccion que dejo a medias y libere SDA. */
#define I2C_BUS_CLEAR_PULSES     9U
#define I2C_BUS_CLEAR_HALF_US    5U

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
static uint8_t  live_override     = 0;  /* LIVE: publicar siempre el valor filtrado */
static uint8_t  level_state       = 0;

static uint8_t  level_on_pending   = 0;
static uint8_t  level_off_pending  = 0;
static uint32_t level_on_tick      = 0;
static uint32_t level_off_tick     = 0;

static uint8_t  i2c_fail_count     = 0;
static uint32_t last_i2c_recover_tick = 0;

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

/* ── Envoltura angular ─────────────────────────────────────────────────
 * El giroscopio integra sin limite y atan2f solo devuelve (-180, 180].
 * Si la innovacion se calcula como (acc - fused) sin envolver, cerca de
 * +-180 aparece una diferencia de ~360 grados: el filtro se queda pegado
 * en 183.7 en vez de pasar a -176.3. Se envuelve el camino angular corto
 * antes de mezclar, no solo la salida. */
static float IMU_Wrap180(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

/* ── Persistencia del cero (TARE) en la ultima pagina de flash ─────────
 * El STM32F103 no tiene EEPROM. Se reserva la ultima pagina de 1 KB
 * (ver STM32F103C8TX_FLASH.ld, LENGTH = 63K) y solo se reescribe cuando
 * el usuario tara o borra el cero. Un corte a mitad de escritura deja
 * el CRC invalido y el arranque cae a tare = 0, sin estado corrupto. */
#define NVM_TARE_ADDR      0x0800FC00UL
#define NVM_TARE_MAGIC     0x54415245UL  /* "TARE" */
#define NVM_TARE_VERSION   1U
#define NVM_TARE_MAX_DEG   360.0f

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float    roll;
    float    pitch;
    uint32_t crc;      /* CRC32 de los 16 bytes anteriores */
} NVM_TareRecord;

static uint32_t NVM_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xEDB88320UL;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

static uint8_t NVM_TareValid(float deg)
{
    /* Descarta NaN, infinito y valores absurdos. */
    return (deg == deg) && (fabsf(deg) <= NVM_TARE_MAX_DEG);
}

/* Lee el cero guardado. Devuelve 1 si habia un registro valido. */
static uint8_t NVM_LoadTare(void)
{
    const NVM_TareRecord *rec = (const NVM_TareRecord *)NVM_TARE_ADDR;

    if (rec->magic != NVM_TARE_MAGIC || rec->version != NVM_TARE_VERSION)
        return 0;

    if (NVM_Crc32((const uint8_t *)rec, sizeof(*rec) - sizeof(uint32_t)) != rec->crc)
        return 0;

    if (!NVM_TareValid(rec->roll) || !NVM_TareValid(rec->pitch))
        return 0;

    tare_roll  = IMU_Wrap180(rec->roll);
    tare_pitch = rec->pitch;
    return 1;
}

static uint8_t NVM_EraseTarePage(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    HAL_StatusTypeDef status;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = NVM_TARE_ADDR;
    erase.NbPages     = 1;

    if (HAL_FLASH_Unlock() != HAL_OK)
        return 0;

    status = HAL_FLASHEx_Erase(&erase, &page_error);

    (void)HAL_FLASH_Lock();

    return (status == HAL_OK) ? 1U : 0U;
}

/* Reescribe la pagina con el cero actual. Solo se llama al tarar. */
static uint8_t NVM_SaveTare(void)
{
    NVM_TareRecord rec;
    uint32_t addr = NVM_TARE_ADDR;
    uint8_t ok = 1;

    rec.magic    = NVM_TARE_MAGIC;
    rec.version  = NVM_TARE_VERSION;
    rec.reserved = 0xFFFFU;
    rec.roll     = tare_roll;
    rec.pitch    = tare_pitch;
    rec.crc      = NVM_Crc32((const uint8_t *)&rec, sizeof(rec) - sizeof(uint32_t));

    if (!NVM_EraseTarePage())
        return 0;

    if (HAL_FLASH_Unlock() != HAL_OK)
        return 0;

    for (uint32_t i = 0; i < (sizeof(rec) / sizeof(uint32_t)); i++)
    {
        uint32_t word = ((const uint32_t *)&rec)[i];

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) != HAL_OK)
        {
            ok = 0;
            break;
        }

        addr += 4U;
    }

    (void)HAL_FLASH_Lock();

    /* Verificar que se escribio lo esperado. */
    if (ok)
    {
        const uint32_t *flash = (const uint32_t *)NVM_TARE_ADDR;
        const uint32_t *ram   = (const uint32_t *)&rec;

        for (uint32_t i = 0; i < (sizeof(rec) / sizeof(uint32_t)); i++)
        {
            if (flash[i] != ram[i])
            {
                ok = 0;
                break;
            }
        }
    }

    return ok;
}

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

    /* Recuperar el cero de la ultima sesion (si el registro es valido). */
    (void)NVM_LoadTare();

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
    live_override    = 0;
    level_state      = 0;

    level_on_pending   = 0;
    level_off_pending  = 0;
    level_on_tick      = HAL_GetTick();
    level_off_tick     = HAL_GetTick();

    i2c_fail_count     = 0;
    last_i2c_recover_tick = HAL_GetTick() - IMU_I2C_RECOVER_COOLDOWN_MS;

    imu_last_tick       = HAL_GetTick();
    last_motion_tick    = HAL_GetTick();
    last_telemetry_tick = 0;

    IMU_ClearOutputs();
}

void IMU_Tare(void)
{
    /* El ángulo absoluto es fused + tare. Al tarar, movemos el cero y lo
     * guardamos envuelto en (-180, 180] para no acumular vueltas. */
    tare_roll  = IMU_Wrap180(tare_roll  + fused_roll);
    tare_pitch += fused_pitch;

    fused_roll  = 0.0f;
    fused_pitch = 0.0f;

    hold_manual     = 0;
    hold_auto       = 0;
    auto_hold_armed = 0;
    level_state     = 0;

    level_on_pending  = 0;
    level_off_pending = 0;

    IMU_Roll   = 0.0f;
    IMU_Pitch  = 0.0f;
    IMU_Level  = 0;
    IMU_Hold   = 0;

    last_motion_tick    = HAL_GetTick();
    last_telemetry_tick = 0;

    /* Persistir el nuevo cero; si falla, el tarado sigue valido en RAM. */
    (void)NVM_SaveTare();
}

uint8_t IMU_TareClear(void)
{
    /* Convertir el cero relativo a absoluto antes de borrar la referencia:
     * la lectura no salta, simplemente deja de restar el offset. */
    fused_roll  = IMU_Wrap180(fused_roll + tare_roll);
    fused_pitch += tare_pitch;

    tare_roll  = 0.0f;
    tare_pitch = 0.0f;

    hold_manual      = 0;
    hold_auto        = 0;
    auto_hold_armed  = 0;
    level_state      = 0;

    level_on_pending  = 0;
    level_off_pending = 0;

    last_telemetry_tick = 0;

    return NVM_EraseTarePage();
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

/**
 * @brief Alterna el modo LIVE: desactiva el auto-hold indefinidamente.
 *
 * Con el auto-hold activo la telemetria repite el ultimo valor congelado,
 * lo que impide medir el ruido real del sensor. En modo LIVE la salida se
 * actualiza siempre con el valor del filtro.
 *
 * Se mantiene aunque se ejecute TARE (el cero cambia, el modo de
 * visualizacion no). Se pierde al reiniciar el IMU o en el auto-off.
 */
void IMU_LiveToggle(void)
{
    live_override = live_override ? 0 : 1;

    if (live_override)
    {
        /* El cambio debe notarse de inmediato: soltar cualquier hold. */
        hold_manual     = 0;
        hold_auto       = 0;
        auto_hold_armed = 0;
    }
    else
    {
        /* Al salir, permitir que el auto-hold vuelva a actuar. */
        auto_hold_armed = 1;
    }

    IMU_Hold = (hold_manual || hold_auto) ? 1U : 0U;
}

uint8_t IMU_LiveState(void)
{
    return live_override;
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

/* ── Recuperacion del bus I2C2 ─────────────────────────────────────────
 *
 * Un esclavo interrumpido a mitad de transaccion puede dejar SDA en bajo
 * y el periferico I2C del STM32 se queda esperando. Reconfigurar el MPU
 * (lo que se hacia antes) no sirve: el problema esta en el bus, no en los
 * registros del sensor. La secuencia correcta es:
 *
 *   1. Apagar I2C2 (HAL_I2C_DeInit) y soltar las lineas.
 *   2. Tomar PB10/PB11 como GPIO open-drain y generar hasta 9 pulsos de
 *      SCL. Cada pulso permite al esclavo sacar un bit; cuando termina la
 *      transaccion suelta SDA.
 *   3. Generar una condicion de STOP manual (SDA sube con SCL alto).
 *   4. Rearmar I2C2 con la configuracion de CubeMX (POD_I2C2_Reinit).
 *   5. Reaplicar la configuracion del MPU, que pudo reiniciarse.
 *
 * El retardo es un lazo aproximado: el bus clear solo necesita ~100 kHz,
 * no un tiempo exacto. */

static void I2C_BusClearDelayUs(uint32_t us)
{
    volatile uint32_t n = us * 14U;
    while (n--)
    {
        __NOP();
    }
}

static void I2C_BusClearPulse(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);
}

/**
 * @brief Ejecuta la recuperacion fisica del bus I2C2.
 * @return 1 si el bus quedo libre y el MPU respondio, 0 si no.
 */
static uint8_t IMU_I2C_Recover(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1. Apagar periferico, interrupciones y pines AF. */
    (void)HAL_I2C_DeInit(&hi2c2);

    /* 2. PB10/PB11 como salida open-drain. El clock de GPIOB sigue
     *    encendido desde el MspInit, pero se asegura por claridad. */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Soltar ambas lineas y comprobar su estado real. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);

    uint8_t scl_ok = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);
    uint8_t sda_ok = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);

    /* 3. Pulsos de SCL hasta que el esclavo libere SDA. */
    for (uint8_t i = 0; i < I2C_BUS_CLEAR_PULSES && !sda_ok; i++)
    {
        I2C_BusClearPulse();
        sda_ok = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    }

    /* 4. STOP manual: SDA baja con SCL alto y luego sube. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    I2C_BusClearDelayUs(I2C_BUS_CLEAR_HALF_US);

    sda_ok = sda_ok && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    scl_ok = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);

    /* 5. Devolver los pines al periferico aunque el bus siga mal: si no,
     *    el I2C queda apagado y no hay ninguna otra via de salida. */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    POD_I2C2_Reinit();

    if (!sda_ok || !scl_ok)
        return 0U;

    /* 6. Comprobar que el MPU responde antes de reescribir cinco registros
     *    con timeout de 100 ms cada uno. Si el sensor no esta, conviene
     *    fallar rapido (~10 ms) y no bloquear el lazo medio segundo. */
    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_I2C_ADDR, 1, 10U) != HAL_OK)
        return 0U;

    if (MPU6050_ConfigModifi(&mpu) != MPU6050_OK)
        return 0U;

    bias_ready = 0;
    bias_count = 0;
    bias_sum_x = 0.0f;
    bias_sum_y = 0.0f;
    bias_sum_z = 0.0f;

    return 1U;
}

static void IMU_I2C_HandleFailure(uint32_t now)
{
    if ((now - last_i2c_recover_tick) < IMU_I2C_RECOVER_COOLDOWN_MS)
        return;

    last_i2c_recover_tick = now;

    if (IMU_I2C_Recover())
        POD_UART_Send("IMU:I2C:RECOVER\r\n");
    else
        POD_UART_Send("IMU:I2C:FAIL\r\n");
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
    {
        i2c_fail_count++;

        if (i2c_fail_count >= IMU_I2C_FAIL_LIMIT)
        {
            i2c_fail_count = 0;

            /* El bus pudo quedar colgado a mitad de transaccion. No basta
             * con reconfigurar el MPU: hay que liberar fisicamente SDA
             * (bus clear) y rearmar I2C2. */
            IMU_I2C_HandleFailure(now);
        }

        return;
    }

    i2c_fail_count = 0;

    float ax_s = accel.ax / ACCEL_SCALE_4G;
    float ay_s = accel.ay / ACCEL_SCALE_4G;
    float az_s = accel.az / ACCEL_SCALE_4G;

    float gx_s = gyro.gx / GYRO_SCALE_250DPS;
    float gy_s = gyro.gy / GYRO_SCALE_250DPS;
    float gz_s = gyro.gz / GYRO_SCALE_250DPS;

    /* ── Validación de muestra ────────────────────────────────────
     * Un golpe, una caída o una lectura corrupta del bus producen
     * magnitudes imposibles. Se descartan sin tocar el filtro.
     * La magnitud no cambia con el remapeo, por eso se valida aquí. */
    float accel_mag_s = sqrtf(ax_s*ax_s + ay_s*ay_s + az_s*az_s);

    if (accel_mag_s < ACCEL_VALID_MIN_G || accel_mag_s > ACCEL_VALID_MAX_G)
    {
        if (!bias_ready)
        {
            /* No calibrar el bias con muestras inválidas. */
            bias_sum_x = 0.0f;
            bias_sum_y = 0.0f;
            bias_sum_z = 0.0f;
            bias_count = 0;
        }

        last_motion_tick   = now;
        last_activity_tick = now;
        moving             = 1;
        hold_auto          = 0;
        auto_hold_armed    = 1;
        IMU_Moving         = 1;

        return;
    }

    /* ── Remapeo por montaje en la caja ───────────────────────────
     * El MPU6050 va sobre una cara lateral larga, de cabeza respecto
     * al montaje original del protoboard. Su eje +X queda vertical y
     * apunta hacia abajo cuando la caja apoya sobre su base.
     *
     * Partiendo del remapeo vertical original:
     *     Xv = +Y_s    Yv = +Z_s    Zv = +X_s
     * se compone un giro de 180° sobre el eje virtual Y para que +Z
     * vuelva a apuntar hacia arriba (det = +1):
     *     Xv = -Y_s    Yv = +Z_s    Zv = -X_s
     *
     * Se aplica igual al acelerómetro y al giroscopio para que el
     * filtro complementario reciba ejes coherentes.
     * ──────────────────────────────────────────────────────────── */
    float ax = -ay_s;
    float ay =  az_s;
    float az = -ax_s;

    float gx = -gy_s;
    float gy =  gz_s;
    float gz = -gx_s;

    float roll_acc  = IMU_Wrap180(atan2f(ay, az) * 180.0f / (float)M_PI - tare_roll);
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI - tare_pitch;

    float accel_mag = accel_mag_s;

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

    /* ── Filtro complementario adaptativo ───────────
     * Se envuelve la prediccion y la innovacion por separado: la mezcla
     * debe usar el camino angular corto. La salida tambien queda en
     * (-180, 180], que es lo que se publica y se dibuja. */
    float alpha = moving ? ALPHA_MOVING : ALPHA_STILL;

    float pred_roll = IMU_Wrap180(fused_roll + GYRO_ROLL_SIGN * gx * dt);

    fused_roll  = IMU_Wrap180(pred_roll +
                              (1.0f - alpha) * IMU_Wrap180(roll_acc - pred_roll));
    fused_pitch = alpha * (fused_pitch + GYRO_PITCH_SIGN * gy * dt)
                + (1.0f - alpha) * pitch_acc;

    /* ── Auto-hold ────────────────────────────────── */
    if (!live_override && auto_hold_armed && !hold_manual && !moving &&
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

    uint8_t in_band  = (abs_roll <= LEVEL_ON_DEG)  && (abs_pitch <= LEVEL_ON_DEG);
    uint8_t out_band = (abs_roll >  LEVEL_OFF_DEG) || (abs_pitch >  LEVEL_OFF_DEG);

    if (level_state)
    {
        /* Para apagar: la salida de banda debe sostenerse en el tiempo. */
        if (out_band)
        {
            if (!level_off_pending)
            {
                level_off_pending = 1;
                level_off_tick    = now;
            }
            else if ((now - level_off_tick) >= LEVEL_OFF_DEBOUNCE_MS)
            {
                level_state       = 0;
                level_off_pending = 0;
            }
        }
        else
        {
            level_off_pending = 0;
        }
    }
    else
    {
        /* Para encender: la entrada a banda debe sostenerse en el tiempo. */
        if (in_band)
        {
            if (!level_on_pending)
            {
                level_on_pending = 1;
                level_on_tick    = now;
            }
            else if ((now - level_on_tick) >= LEVEL_ON_DEBOUNCE_MS)
            {
                level_state      = 1;
                level_on_pending = 0;
            }
        }
        else
        {
            level_on_pending = 0;
        }
    }

    IMU_Level = level_state;

    /* ── Telemetría ───────────────────────────────── */
    if ((now - last_telemetry_tick) >= IMU_TELEMETRY_MS)
    {
        last_telemetry_tick = now;

        int16_t roll_i  = (int16_t)(IMU_Roll  * 10.0f);
        int16_t pitch_i = (int16_t)(IMU_Pitch * 10.0f);
        char msg[MSG_LEN];

        snprintf(msg, MSG_LEN, "IMU:L%d,R%d,P%d,H%d,M%d\r\n",
                 (int)IMU_Level, roll_i, pitch_i, (int)IMU_Hold, (int)IMU_Moving);
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
        live_override   = 0;
        level_state     = 0;

        level_on_pending  = 0;
        level_off_pending = 0;

        IMU_ClearOutputs();
        last_activity_tick = now;

        POD_UART_Send("IMU:OFF\r\n");
    }
}
