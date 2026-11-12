# Recuperación del bus I2C2

El MPU6050 cuelga de **I2C2** (`PB10` = SCL, `PB11` = SDA). Si una
transacción se interrumpe a mitad —ruido en los cables, un dupont flojo, un
golpe, el sensor reiniciándose— el esclavo puede quedarse **reteniendo SDA en
bajo**. El periférico I2C del STM32 entonces espera una condición que nunca
llega y todas las lecturas siguientes fallan.

Antes de este cambio, tras 5 fallos seguidos el firmware solo volvía a escribir
los registros de configuración del MPU y emitía `IMU:I2C:RECOVER`. Eso no
resuelve el problema: **el fallo está en el bus físico, no en los registros del
sensor**. Si SDA sigue en bajo, ninguna escritura de registro puede pasar.

---

## Qué hace ahora

La recuperación se dispara tras **5 fallos consecutivos** de
`MPU6050_GetMotion6()` (`IMU_I2C_FAIL_LIMIT`), es decir, unos 250 ms a 20 Hz.
Después hay un **cooldown de 1 s** (`IMU_I2C_RECOVER_COOLDOWN_MS`) para no
martillear el periférico si el bus está realmente muerto.

Secuencia en `IMU_I2C_Recover()` (`pod_imu.c`):

1. **`HAL_I2C_DeInit(&hi2c2)`** — apaga el periférico, sus interrupciones y
   devuelve `PB10`/`PB11` a GPIO.
2. **Pines como salida open-drain** — se sueltan ambas líneas y se leen para
   saber si el esclavo las libera.
3. **Hasta 9 pulsos de SCL** — el estándar I2C permite al esclavo terminar la
   transacción pendiente: cada pulso le deja soltar un bit. Se para en cuanto
   SDA sube.
4. **STOP manual** — SDA baja con SCL alto y luego sube. Deja al esclavo en un
   estado conocido.
5. **`POD_I2C2_Reinit()`** — vuelve a llamar a `MX_I2C2_Init()`, que rearma el
   periférico y los pines en modo alterno open-drain con la configuración de
   CubeMX. Se hace **aunque el bus no se haya liberado**: si no, el I2C queda
   apagado y no hay otra vía de salida.
6. **Sonda de presencia y `MPU6050_ConfigModifi(&mpu)`** — primero
   `HAL_I2C_IsDeviceReady` con timeout corto (~10 ms); solo si el sensor
   responde se reescriben DLPF, escalas y sample rate. Si no está, se falla
   rápido en vez de bloquear el lazo ~500 ms con cinco escrituras.
7. **Recalibrar el bias del giroscopio** — tras una recuperación no se confía
   en la integración hasta volver a medir el bias en reposo (~2.5 s).

### Mensajes

| Mensaje | Significado |
|---|---|
| `IMU:I2C:RECOVER` | Bus liberado y MPU reconfigurado |
| `IMU:I2C:FAIL` | SDA/SCL siguen retenidos o el MPU no respondió |

El ESP32 los traduce a `[POD 042] I2C RECOVER` / `[POD 042] I2C FAIL` y los
registra en el log serie.

---

## Por qué un bus clear y no solo reinicializar

`HAL_I2C_DeInit` + `HAL_I2C_Init` rearma el periférico, pero **no obliga al
esclavo a soltar SDA**. Si el MPU quedó a mitad de una transacción, seguirá
esperando relojes. Los pulsos de SCL son la única forma de sacarlo de ese
estado sin cortar la alimentación.

Por eso el orden importa: primero se libera el bus con GPIO, después se rearma
el periférico. Al revés se vuelve al mismo bloqueo.

## Por qué `MX_I2C2_Init` no se tocó

`MX_I2C2_Init()` es código generado por CubeMX y sigue siendo `static`. En
lugar de exponerlo, se añadió `POD_I2C2_Reinit()` en un bloque `USER CODE` de
`main.c`, con su prototipo en `USER CODE BEGIN EFP` de `main.h`. Así:

- CubeMX puede regenerar `MX_I2C2_Init` sin borrar el punto de entrada.
- Hay una sola fuente de verdad para la configuración del periférico.

## Propagación de errores del MPU

`MPU6050_ConfigModifi()` antes ignoraba todos los errores de
`MPU6050_WriteReg()` y siempre devolvía `MPU6050_OK`. Ahora devuelve
`MPU6050_ERROR` en la primera escritura que falle. Sin eso, la recuperación no
podía distinguir "bus libre" de "bus libre pero el sensor no responde".

---

## Límites

- **No puede arreglar SCL retenido en bajo** por el esclavo: si tras soltar la
  línea sigue en bajo, se reporta `IMU:I2C:FAIL`. Eso ya es hardware.
- **No sustituye el montaje.** El bus clear recupera el estado lógico; si un
  cable hace contacto intermitente, el fallo volverá. La solución de fondo son
  cables cortos/rígidos y pegamento térmico en los conectores.
- **No se probó en hardware todavía.** La lógica es la secuencia estándar de
  *bus recovery* de I2C, pero hay que provocar el fallo para validarla.

## Cómo verificarlo en hardware

1. Con el IMU activo, desconectar y reconectar **SDA** (o SCL) del MPU
   brevemente mientras el firmware lee.
2. En el terminal deben aparecer 5 fallos (~250 ms) y luego:
   - `IMU:I2C:RECOVER` si el bus se liberó, o
   - `IMU:I2C:FAIL` si quedó retenido.
3. Si fue `RECOVER`, la telemetría `IMU:L,R,P,H,M` debe volver en ~2.5 s
   (recalibración de bias).
4. Repetir con el cable desconectado del todo: debe salir `FAIL` una vez por
   segundo, sin bloquear el resto del firmware (LED, UART, botón).
