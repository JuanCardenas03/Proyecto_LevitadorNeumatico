# Pruebas en el PC del firmware de la PSoC

Requiere `gcc` (Linux o WSL) y, para la gráfica, Python con matplotlib. Desde esta carpeta, con el firmware en `../`:

```bash
./correr.sh
```

| Archivo | Qué prueba |
|---|---|
| `equivalencia.c` | El PID de `control.c` contra la transcripción literal del `.ino` de la ESP32: mismo duty en cada paso |
| `sim_host.c` | `main.c` y todos los módulos, sin cambios, sobre APIs falsas de PSoC Creator (`stubs/project.h`): HC-SR04 emulado a nivel de pines, ventilador + pelota simulados, ESP32 sondeando cada 500 ms, comandos por consola |
| `grafica.py` | Dibuja `traza.csv` → `simulacion_firmware.png` |

Salidas: `log_firmware.txt` (lo que la PSoC escribiría por el KitProg) y `traza.csv`.
