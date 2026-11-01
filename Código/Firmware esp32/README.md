# Levitador Neumático — Firmware ESP32 (maestro Modbus RTU)

Firmware de la ESP32 del proyecto **Levitador Neumático con control PID en PSoC 5LP**. La ESP32 es el puente entre el HMI (LabVIEW) y el controlador (PSoC):

- Recibe **comandos de texto** de LabVIEW por USB y los traduce a escrituras **Modbus RTU** hacia la PSoC.
- Lee la **telemetría** de la PSoC cada 100 ms y la reenvía a LabVIEW en una línea de texto.
- Ofrece un **puerto de depuración** independiente con los valores ya des-escalados.

```
 LabVIEW (HMI)  <--USB, 115200 8N1, texto-->  ESP32  <--RS485, 19200 8N1, Modbus RTU-->  PSoC 5LP (PID)
                                                |
                                       UART1 (GPIO25/26)
                                        depuración (USB-TTL)
```

## Estado

| Etapa | Estado |
|---|---|
| Comunicación bidireccional con LabVIEW (comandos y telemetría) | Funciona, probada contra un esclavo Modbus simulado |
| Integración con la PSoC real | Pendiente (la PSoC aún no está programada) |

## Hardware y conexiones

| Señal | Pin ESP32 | Nota |
|---|---|---|
| DI del MAX485 | GPIO17 (TX2) | Datos que transmite la ESP32 |
| RO del MAX485 | GPIO16 (RX2) | Datos que recibe la ESP32 |
| DE + RE del MAX485 (unidos) | GPIO4 | Dirección half-duplex: HIGH al transmitir, LOW al recibir |
| Depuración TX | GPIO25 (UART1) | A un adaptador USB-TTL aparte |
| Depuración RX | GPIO26 (UART1) | Sin uso por ahora |

## Puertos y parámetros

| UART | Uso | Parámetros |
|---|---|---|
| UART0 (`Serial`, USB) | Protocolo de texto con LabVIEW. **Solo** comandos de entrada y líneas `DATA,` de salida | 115200, 8N1 |
| UART1 (`DebugSerial`) | Depuración humana | 115200, 8N1 |
| UART2 (`Serial2`) | Bus RS485 / Modbus RTU hacia la PSoC | 19200, 8N1, esclavo con dirección **1** |

## Mapa de registros Modbus

Las alturas, el setpoint y el error están en **cm × 10** (226 = 22.6 cm).

### Holding registers (escritura FC06)

| Dirección | Nombre | Escala / rango |
|---|---|---|
| 0x0000 | Setpoint | cm × 10, 0–1000 |
| 0x0001 | Kp | × 100 |
| 0x0002 | Ki | × 100 |
| 0x0003 | Kd | × 100 |
| 0x0004 | Modo | 0 = manual, 1 = automático |
| 0x0005 | PWM manual | 0–1000 (= 0.0–100.0 %) |

### Input registers (lectura FC04): mapa ampliado respecto a la guía

La ESP32 lee los **7 registros en una sola transacción** (dirección 0, cantidad 7), en el mismo orden que la trama `DATA,`:

| Dirección | Nombre | Escala |
|---|---|---|
| IR 0 | Setpoint vigente de la planta | cm × 10 |
| IR 1 | Altura medida | cm × 10 |
| IR 2 | PWM aplicado | 0–1000 |
| IR 3 | Error | cm × 10, **con signo** |
| IR 4 | Estado | bitmap |
| IR 5 | Tramas Modbus válidas | contador |
| IR 6 | Errores de CRC | contador |

> **Cambio respecto a la guía (acordado con el equipo):** la guía define 6 input registers (altura a errores de CRC). Aquí se agrega el setpoint como **IR 0** y todos los demás corren una posición. **La PSoC debe exponer en IR 0 una copia de solo lectura del holding 0x0000**, de modo que lo que ve LabVIEW sea el setpoint que realmente tiene la planta. La Sección 4.4 de la guía debe actualizarse.

### Coils (escritura FC05)

| Dirección | Nombre | Efecto |
|---|---|---|
| 0x0000 | Habilitar control | 1 = control activo, 0 = apagado |
| 0x0001 | Reiniciar integrador | Pulso: reinicia el término integral del PID |
| 0x0002 | Paro de emergencia | 1 = emergencia activada |

## Protocolo con LabVIEW

### Comandos (LabVIEW → ESP32)

Una línea por comando: `CLAVE:VALOR` terminada en salto de línea (`\n`, LF, `0x0A`). El valor es un entero ya escalado. Varios comandos pueden ir en una misma escritura.

| Comando | Rango aceptado | Se escribe en | Nota |
|---|---|---|---|
| `SP:300` | 0–1000 (30.0 cm) | Holding 0x0000 | Fuera de rango: se recorta |
| `KP:40` | 0–65535 (0.40) | Holding 0x0001 | Se recorta |
| `KI:104` | 0–65535 (1.04) | Holding 0x0002 | Se recorta |
| `KD:22` | 0–65535 (0.22) | Holding 0x0003 | Se recorta |
| `MODE:1` | 0 o 1 | Holding 0x0004 | Otro valor se interpreta como 1 |
| `PWM:500` | 0–1000 (50.0 %) | Holding 0x0005 | Se recorta |
| `EN:1` | 0 o 1 | Coil 0x0000 | Otro valor se interpreta como 1 |
| `EMG:1` | 0 o 1 | Coil 0x0002 | Otro valor se interpreta como 1 |
| `RST:1` | cualquiera | Coil 0x0001 | Pulso: ON, 50 ms, OFF |

Las claves no distinguen mayúsculas de minúsculas. Aunque LabVIEW ya limita los rangos, la ESP32 los vuelve a validar como segunda barrera; cada ajuste deja un aviso en el puerto de depuración.

**Reglas del parser:**
- Terminador: `\n` o `\r`. Con `\r\n` la segunda mitad se ignora.
- Las líneas vacías se ignoran sin error (las tandas de LabVIEW empiezan con un `\n` suelto).
- Una línea **sin terminador** se procesa igualmente tras **30 ms** sin bytes nuevos. Un comando partido en dos tramos cercanos se arma completo.
- Si el salto de línea llega escrito como texto (barra + `n`), se ignora desde la barra y se avisa.
- Las líneas de más de 64 caracteres se descartan completas.
- En todos los casos anómalos se deja un aviso en el puerto de depuración; conviene corregirlos en el VI, aunque el firmware los tolere.

### Telemetría (ESP32 → LabVIEW)

Cada 100 ms (10 Hz), en una sola escritura al puerto y terminada **solo con `\n`**:

```
DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc
```

| Campo | Escala | Des-escalado en el HMI |
|---|---|---|
| `sp` | cm × 10 | ÷ 10 → cm |
| `altura` | cm × 10 | ÷ 10 → cm |
| `pwm` | 0–1000 | ÷ 10 → % |
| `error` | cm × 10, con signo | ÷ 10 → cm |
| `estado` | bitmap | — |
| `tramas_ok` | contador | — |
| `errores_crc` | contador | — |

Ejemplo: `DATA,300,226,149,74,3,1520,2` → SP 30.0 cm, altura 22.6 cm, PWM 14.9 %, error 7.4 cm.

## Comportamiento

- **Ciclo de telemetría:** una única transacción FC04 de 7 registros cada 100 ms.
- **Falla de lectura Modbus:** ese ciclo **no** se envía `DATA,` y el error queda en el puerto de depuración. LabVIEW lo percibe como ausencia de tramas.
- **Timeout del maestro:** el de la librería ModbusMaster, **2000 ms**, sin modificar. Si el esclavo no responde, el `loop()` espera hasta 2 s por transacción: en ese tiempo no salen tramas ni se procesan comandos. El código de error es `0xE2`.
- **Silencio de LabVIEW:** se mantiene el último setpoint. La ESP32 solo escribe en la PSoC cuando llega un comando; la telemetría sigue leyéndose y enviándose. La PSoC conserva además su propio timeout de seguridad por Modbus.
- **`RST`:** el pulso usa un `delay(50)`, de modo que el `loop()` se bloquea 50 ms. Es aceptable por ser un comando poco frecuente.

## Puerto de depuración (UART1)

Muestra los valores ya des-escalados. Formatos principales:

```
[TEL] SP=30.0cm Alt=22.6cm PWM=14.9% Err=7.4cm Est=0b000011 OK=1520 CRC=2 | Modbus=15ms Periodo=100ms
[CMD] Recibido: SP = 300
  [TX] Holding 0x0 (Setpoint) = 30.0 cm [crudo=300] -> OK
[VALIDACION] SP fuera de rango (1500) -> ajustado a 1000
[CMD] Linea sin terminador (falta '\n'); se procesa por inactividad
[RX] ERROR leyendo input registers 0..6, codigo 0xE2
```

- `[TEL]` sale cada 500 ms (no en cada ciclo, para no bloquear el `loop()`). `Modbus` es lo que tardó la lectura del último ciclo y `Periodo` el tiempo real entre ciclos.
- Código `0xE2` = timeout de respuesta.

## Compilación

1. Arduino IDE con el soporte para placas ESP32 (core de Espressif).
2. Instalar la librería **ModbusMaster** de Doc Walker, versión 2.0.1 (Gestor de librerías).
3. Guardar el sketch como `levitador_esp32_maestro/levitador_esp32_maestro.ino` (en Arduino la carpeta debe llamarse igual que el sketch).
4. Seleccionar la placa ESP32 y cargar.

## Pruebas sin la PSoC

Se usa un esclavo Modbus RTU simulado (el simulador propio en Python/Tkinter, o *Modbus Slave* de modbustools.com) en un PC con adaptador USB-RS485:

- Dirección de esclavo 1, Modbus RTU, 19200, 8N1.
- Input registers 0–6, holding registers 0–5 y coils 0–2.
- El simulador debe **espejar el holding 0 en el input register 0**, como lo hará la PSoC; si no, el `sp` de la trama no cambiará al enviar `SP:`.

Para probar el lado de LabVIEW, enviar comandos por USB y verificar en el simulador que el registro o coil correspondiente cambia de inmediato.

## Pendiente

- [ ] Implementar en la PSoC el mapa ampliado (IR 0–6, con IR 0 como espejo del holding 0x0000) y actualizar la Sección 4.4 de la guía.
- [ ] Pruebas con la PSoC real (MAX485 #2): direcciones, CRC y tiempos de respuesta.
- [ ] Prueba de resistencia: 30 minutos continuos sin que crezcan los errores de CRC.
- [ ] Confirmar que el HMI acepta el campo `error` con signo.
- [ ] Opcional: definir una trama o indicador de enlace caído hacia LabVIEW (hoy simplemente no se envía `DATA,`).
- [ ] Opcional: reemplazar el `delay(50)` de `RST` por una máquina de estados no bloqueante.

## Archivos

| Archivo | Descripción |
|---|---|
| `levitador_esp32_maestro.ino` | Firmware de la ESP32 |
| `README.md` | Este documento |
