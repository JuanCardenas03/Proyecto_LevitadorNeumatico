# Pruebas en el PC del firmware de la PSoC

Requiere `gcc` (Linux o WSL) y, para la gráfica, Python 3 con matplotlib. Las fuentes del firmware se toman de `../Levitador.cydsn/` (variable `FW` de `correr.sh`). Desde esta carpeta:

```bash
./correr.sh        # o: sh correr.sh
```

| Archivo | Qué prueba |
|---|---|
| `equivalencia.c` | El PID de `control.c` contra la transcripción literal del `.ino` de la ESP32 (`Codigo/PID/levitador_esp32_PID/`), ambos con los parámetros de arranque de `control.h`: mismo duty en cada paso |
| `sim_host.c` | `main.c` y todos los módulos, sin cambios, sobre APIs falsas de PSoC Creator (`stubs/project.h`): HC-SR04 emulado a nivel de pines, ventilador + pelota simulados, ESP32 sondeando cada 500 ms, comandos por consola |
| `grafica.py` | Dibuja `traza.csv` → `simulacion_firmware.png` |

Resultado esperado con el firmware final:

```
EQUIVALENCIA: PASA        (diferencia máxima de duty 0.0 % en 3000 pasos)
TOTAL: 28 pasan, 0 fallan
```

Salidas: `log_firmware.txt` (lo que la PSoC escribiría por el KitProg), `traza.csv` y `simulacion_firmware.png`. Los ejecutables `equivalencia` y `sim_host` que genera el script no se versionan (`.gitignore`).
