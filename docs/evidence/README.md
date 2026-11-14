# Evidencia de validación

Esta carpeta guarda las capturas **reales** de las pruebas de hardware del
POD-042. Su objetivo es que cualquiera pueda comprobar lo que afirma el README
sin tener que confiar en la palabra del autor.

## Reglas

1. **Solo capturas reales.** Nada de ejemplos inventados, logs editados a mano ni
   resultados "esperados". Si una prueba no se hizo, no hay archivo.
2. **Un archivo por prueba.** Nombre con fecha, prueba y fuente:
   `YYYY-MM-DD_<prueba>_<fuente>.log`
   Ejemplo: `2026-11-15_roll180_uart.log`.
3. **Anotar el contexto** al inicio del archivo, como comentarios `#`:
   - commit del STM32 (`git rev-parse --short HEAD`) y del ESP32;
   - fecha y hora;
   - montaje usado (caja, cables, superficie);
   - comando o acción realizada.
4. **No borrar los fallos.** Un log donde algo salió mal también es evidencia; se
   documenta y se explica.
5. **Texto plano.** Nada de capturas de pantalla como única prueba; si hay foto o
   video, se puede enlazar, pero el log manda.

## Plantilla

```text
# Prueba: roll ±180°
# Fecha: 2026-11-15 18:30
# STM32: 92a97f9
# ESP32: 393012d
# Montaje: caja 5x8x2.5 cm, MPU en cara lateral
# Acción: rotación lenta de 150° a 220° sobre el eje de roll
# Resultado esperado: pasar de +179.x a -179.x sin corregir 360°

IMU:L0,R1792,P03,H0,M1
IMU:L0,R1798,P03,H0,M1
IMU:L0,R-1796,P04,H0,M1
...
```

## Pruebas a capturar

| Prueba | Archivo sugerido | Estado |
|---|---|---|
| Roll cruzando ±180° | `YYYY-MM-DD_roll180_uart.log` | Pendiente |
| Recuperación I2C (SDA desconectada) | `YYYY-MM-DD_i2c_recover_uart.log` | Pendiente |
| I2C con cable fuera (`FAIL` ~1/s) | `YYYY-MM-DD_i2c_fail_uart.log` | Pendiente |
| Cadencia sin OLED, 60 s | `YYYY-MM-DD_cadence60s_uart.log` | Pendiente |
| Nivel + histéresis + LED | `YYYY-MM-DD_level_uart.log` | Pendiente |
| `TARE` / `TARECLR` | `YYYY-MM-DD_tare_ble.log` | Pendiente |
| Batería | `YYYY-MM-DD_batt_uart.log` | Pendiente |

## Cómo capturar (ESP32)

```bash
source ~/esp/esp-idf/export.sh
idf.py -p /dev/ttyACM0 monitor | tee docs/evidence/2026-11-15_roll180_uart.log
```

Cerrar con `Ctrl + ]`. Si se abre el puerto con Python, usar
`ser.dtr = False; ser.rts = False` para no reiniciar el ESP32.
