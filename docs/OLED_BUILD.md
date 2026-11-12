# OLED fuera del build: por qué y cómo

El flag `POD_USE_OLED` (en `Drivers/Drivers_POD/pod_shared.h`) decide si la
interfaz OLED forma parte del firmware. Por defecto vale **0**: el código
sigue en el repositorio y se sigue compilando, pero no se inicializa, no se
refresca y el linker lo descarta del binario.

Este documento explica la decisión. No es una preferencia estética: hay
cuatro razones, y la de rendimiento no es la más importante.

---

## 1. Comparte bus con el sensor

- `Drivers/OLED/ssd1306_conf.h` define `SSD1306_I2C_PORT = hi2c2`.
- El MPU6050 también cuelga de **I2C2** (`PB10`/`PB11`).
- `Drivers/OLED/ssd1306.c` usa `HAL_MAX_DELAY` en **todas** las escrituras
  (`HAL_I2C_Mem_Write`) y en las esperas de flags.

`HAL_MAX_DELAY` es espera **infinita**. Si el panel no responde, deja SDA en
bajo o no libera el bus, el firmware se queda bloqueado en el mismo bus del
que depende el sensor de orientación. La OLED de este prototipo está muerta:
mantener sus llamadas es conservar un modo de fallo que ya se manifestó como
`IMU:I2C:RECOVER`.

Quitar la OLED del build no mitiga ese riesgo: lo elimina.

## 2. Consumo de tiempo del lazo

Cada `ssd1306_UpdateScreen()` recorre las páginas del panel:

```
4 páginas × (3 bytes de comando + 128 bytes de datos) = 524 bytes
524 × 9 bits / 400 000 Hz ≈ 11.8 ms  (más overhead de 16 transacciones HAL)
```

La pantalla de nivel (`POD_UI_LevelScreen`) se refresca **cada 100 ms**
mientras el IMU está activo. Son ~12 ms bloqueado de cada 100 ms: en torno al
**12 % del tiempo** dibujando una pantalla que no se ve, en el mismo lazo que
muestrea a 20 Hz y publica telemetría a 10 Hz.

Durante el arranque, la animación de carga se refresca cada 40 ms durante
~2.5 s, con el mismo coste por refresco.

## 3. Tamaño del binario

Es la razón menos importante hoy (el firmware usa ~37 % de la flash), pero es
medible y no trivial:

| Configuración | text | data | bss | Flash (text+data) |
|---|---:|---:|---:|---:|
| `POD_USE_OLED = 1` | 35 556 | 100 | 3 056 | **35 656 B** (54.4 %) |
| `POD_USE_OLED = 0` | 23 876 | 100 | 2 520 | **23 976 B** (36.6 %) |

El ahorro real es **11 680 B de flash y 536 B de RAM**, bastante mayor que el
bloque `ui_oled.c + ssd1306.c` aislado. La razón: al desaparecer la animación
de carga también se van las tablas de fuentes/iconos y la ruta de `sin()`
en doble precisión que arrastraba parte de `libm` en un Cortex-M3 sin FPU.

## 4. Honestidad del entregable

Un build que incluye rutas para hardware ausente no describe el aparato real.
Con el flag, el README puede decir "compilado sin pantalla" y el día que la
pantalla se repare basta poner el flag en 1. Eso es gestión de configuración,
no código muerto.

---

## Cómo funciona

El flag se define una sola vez y se puede sobrescribir desde la línea de
compilación porque está protegido con `#ifndef`:

```c
#ifndef POD_USE_OLED
#define POD_USE_OLED 0
#endif
```

Se aplica en los **puntos de llamada**, no dentro de `ui_oled.c`:

| Archivo | Qué se condiciona |
|---|---|
| `Core/Src/main.c` | `ssd1306_Init()` y `POD_UI_TASK()` |
| `Drivers/Drivers_POD/pod_fsm.c` | `POD_UI_TASK()` (cuerpo), `Clear_Screen_ssd1306()` y `ssd1306_SetDisplayOn()` en sleep/wake |

`ui_oled.c` y `ssd1306.c` **se siguen compilando siempre**. Eso mantiene la
cobertura del compilador: si alguien rompe la API de la OLED, la build falla
aunque el flag esté en 0. El linker (`-Wl,--gc-sections`, ya activo en
Release) descarta las secciones al no tener referencias.

## Qué no es

- **No borra el código.** La pantalla, la burbuja, la batería y la animación
  siguen en el repositorio y compilando.
- **No está probado que arregle la telemetría a ~5 Hz.** Es una hipótesis:
  UI y telemetría comparten la cadencia de 100 ms. Si tras desactivar la OLED
  la cadencia sigue baja, el cuello de botella está en otra parte (UART, BLE,
  cola de comandos) y hay que medirlo aparte.
- **No sustituye la recuperación real del bus I2C.** Si el MPU se cuelga solo,
  sigue faltando `HAL_I2C_DeInit` + `MX_I2C2_Init` o un *bus clear*.

## Cómo activar la pantalla

1. Reparar o conectar el panel SSD1306 (I2C2, dirección `0x3C`).
2. Cambiar `POD_USE_OLED` a **1** en `pod_shared.h`.
3. Compilar y flashear.

Si el panel vuelve a fallar, volver a 0. El cero de `TARE` guardado en flash
no se ve afectado.

## Cómo verificar el efecto en hardware

1. Con el flag en 1, capturar 60 s de telemetría y contar las líneas `IMU:L`
   (cadencia actual ~5 Hz).
2. Repetir con el flag en 0.
3. Comparar también el tamaño y, si se puede, el *jitter* entre muestras.

La cadencia no se puede medir desde el binario: hay que capturarla en el
puerto serie. Ver `POD-042_LOG_2026-11-10.md` para las capturas previas.
