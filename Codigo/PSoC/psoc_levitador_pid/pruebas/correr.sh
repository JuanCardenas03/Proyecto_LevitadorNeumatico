#!/bin/sh
# Compila y corre las dos pruebas del firmware en el PC.
set -e
FW=..
CF="-std=gnu99 -O2 -Wall -DPRUEBA_HOST -Istubs -I$FW"

gcc $CF equivalencia.c $FW/control.c $FW/modbus_slave.c $FW/planta_modelo.c -lm -o equivalencia
echo "== Prueba 1: equivalencia con levitador_esp32_PID.ino"
./equivalencia

gcc $CF sim_host.c $FW/main.c $FW/modbus_slave.c $FW/control.c $FW/hw_levitador.c \
    $FW/consola.c $FW/debug_log.c $FW/planta_modelo.c -lm -o sim_host
echo
echo "== Prueba 2: firmware completo simulado (107 s)"
./sim_host

python3 grafica.py traza.csv 2>/dev/null && echo "Grafica: simulacion_firmware.png" || true
