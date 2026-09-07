# Levitador Neumático — Firmware ESP32 (Maestro Modbus RTU)

Firmware de la ESP32 dentro del proyecto de **Levitador Neumático con Control PID en PSoC 5LP**. Este componente actúa como **maestro Modbus RTU** entre el HMI (LabVIEW) y el controlador (PSoC 5LP), traduciendo comandos de texto en tramas Modbus y reportando telemetría de vuelta.

Este README documenta el avance del frente de ESP32 y se actualizará a medida que el firmware evolucione.

## Estado actual: Fase 1 — ESP32 sola, sin LabVIEW ni PSoC

En esta etapa se está validando la cadena Modbus completa (UART, control de dirección RS485, CRC) **sin depender de la PSoC**. En su lugar:

- El "esclavo" Modbus es un **PC corriendo Modbus Slave** ([modbustools.com](https://www.modbustools.com/download.html)), conectado vía un **adaptador USB-RS485**.
- Los comandos que después vendrán de LabVIEW por USB se están simulando **escribiéndolos a mano por el monitor serie**, usando la misma sintaxis de texto que usará el HMI.

Esto permite avanzar el firmware de la ESP32 sin quedar bloqueados esperando a que la PSoC esté lista.

## Hardware y conexiones

| Señal MAX485 | Pin ESP32 | Nota |
|---|---|---|
| DI (Driver In) | GPIO17 (TX2) | Datos que la ESP32 transmite |
| RO (Receiver Out) | GPIO16 (RX2) | Datos que la ESP32 recibe |
| DE + RE (unidos) | GPIO4 | Control de dirección half-duplex |

Se usa **Serial2 (UART2)** para el bus RS485, dejando **Serial (USB, UART0)** libre para el protocolo de texto (monitor serie por ahora, LabVIEW en la Fase 2).

## Parámetros de comunicación

| Enlace | Parámetros |
|---|---|
| Bus RS485 / Modbus RTU | 19200 baudios, 8N1, dirección de esclavo = 1 |
| USB (texto, monitor serie / futuro LabVIEW) | 115200 baudios, líneas terminadas en `\n` |

## Mapa de registros Modbus (contrato con la PSoC)

### Holding Registers (FC 06 / FC 16 — escritura)

| Dirección | Nombre | Escala |
|---|---|---|
| 0x0000 | Setpoint de altura | mm × 10 |
| 0x0001 | Kp | × 100 |
| 0x0002 | Ki | × 100 |
| 0x0003 | Kd | × 100 |
| 0x0004 | Modo (0=manual, 1=auto) | — |
| 0x0005 | PWM manual | 0–1000 |

### Input Registers (FC 04 — solo lectura)

| Dirección | Nombre | Escala |
|---|---|---|
| 0x0000 | Altura medida | mm × 10 |
| 0x0001 | PWM aplicado | 0–1000 |
| 0x0002 | Error actual (con signo) | mm × 10 |
| 0x0003 | Palabra de estado | bitmap |
| 0x0004 | Tramas Modbus válidas | contador |
| 0x0005 | Errores de CRC | contador |

### Coils (FC 01 / FC 05)

| Dirección | Nombre | Efecto |
|---|---|---|
| 0x0000 | Habilitar control | 1 = control activo, 0 = ventilador apagado |
| 0x0001 | Reset integrador | pulso: reinicia el término integral del PID |
| 0x0002 | Paro de emergencia | 1 = apaga todo inmediatamente |

> El mapa completo, incluyendo la palabra de estado bit a bit, vive en la guía original del proyecto (`Guia_ESP32_HMI_Modbus.pdf`). Ese documento es la fuente de verdad; este README es un resumen operativo.

## Protocolo de comandos de texto (simulando a LabVIEW)

Formato: `CLAVE:VALOR`, una línea terminada en `\n`.

| Comando | Significado | Se traduce a |
|---|---|---|
| `SP:300` | Setpoint = 30.0 cm | Holding 0x0000 = 3000* |
| `KP:40` | Kp = 0.40 | Holding 0x0001 = 40 |
| `KI:104` | Ki = 1.04 | Holding 0x0002 = 104 |
| `KD:22` | Kd = 0.22 | Holding 0x0003 = 22 |
| `MODE:1` | 1=automático, 0=manual | Holding 0x0004 |
| `PWM:500` | Duty manual = 50.0 % | Holding 0x0005 = 500 |
| `EN:1` | Habilitar control | Coil 0x0000 |
| `EMG:1` | Paro de emergencia | Coil 0x0002 |
| `RST:1` | Reiniciar integrador del PID | Coil 0x0001 (pulso) |

\* El valor ya viene escalado como entero (igual que el registro Modbus); la ESP32 solo lo copia, sin recalcular.

La ESP32 responde por el mismo puerto con una línea de telemetría:

```
DATA:altura,pwm,error,estado,tramas_ok,errores_crc
```

## Decisiones y acuerdos tomados hasta ahora

- **Timeout de LabVIEW:** si la ESP32 deja de recibir comandos, **mantiene el último setpoint** (no apaga el sistema). La PSoC tiene su propio timeout de seguridad por Modbus, independiente de esto.
- **`RST` como pulso:** al recibir `RST:1`, la ESP32 escribe el coil en `ON` y lo regresa a `OFF` ~50 ms después, para generar un flanco de subida en vez de dejar el coil sostenido.
- **Cadencia de telemetría:** en esta fase de pruebas manuales se lee cada **1000 ms** (1 Hz), para que el monitor serie sea legible mientras se escriben comandos a mano. La especificación final (Sección 3.3) pide **100 ms (10 Hz)** una vez que el otro extremo sea LabVIEW real — pendiente de cambiar en la Fase 2.
- **Librería Modbus:** [ModbusMaster de Doc Walker](https://github.com/4-20ma/ModbusMaster) v2.0.1, por su simplicidad y soporte estándar en Arduino/ESP32.

## Cómo probar este firmware

1. Cablear el MAX485 según la tabla de pines de arriba, y conectarlo al adaptador USB-RS485 del PC.
2. Abrir **Modbus Slave**, configurar el puerto serie (19200, 8N1, RTU), dirección de esclavo = 1, y crear bloques de **Holding Registers** (0–5) y **Coils** (0–2).
3. Cargar `firmware_esp32_modbus_test.ino` en la ESP32 y abrir el monitor serie (115200 baudios, fin de línea en "Newline").
4. Escribir comandos como `SP:300`, `KP:40`, `EN:1`, `RST:1` y verificar que los valores cambian del lado de Modbus Slave.
5. Confirmar que la línea `DATA:...` se imprime periódicamente con la telemetría leída desde los Input Registers.

## Pendiente / próximos pasos (Fase 2 y 3)

- [ ] Subir la cadencia de telemetría a 100 ms una vez integrado con LabVIEW.
- [ ] Reemplazar el monitor serie por la comunicación real vía puerto VISA con LabVIEW.
- [ ] Validar rangos de entrada antes de escribir (p. ej. `MODE` solo 0/1) — pendiente de decidir si se agrega.
- [ ] Reemplazar el adaptador USB-RS485 por el MAX485 #2 conectado a la PSoC real (Fase 3 de la guía).
- [ ] Prueba de resistencia: 30 minutos de operación continua sin errores de CRC crecientes.

## Archivos de este frente

| Archivo | Descripción |
|---|---|
| `firmware_esp32_modbus_test.ino` | Sketch actual: maestro Modbus + parser de comandos de texto por monitor serie, para pruebas con Modbus Slave. |
