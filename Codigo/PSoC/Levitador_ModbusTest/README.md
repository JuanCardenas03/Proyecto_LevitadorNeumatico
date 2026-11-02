# PSoC 5LP – Firmware de prueba: esclavo Modbus RTU con depuración por KitProg

Levitador neumático · CY8CKIT-059 · PSoC Creator 4.4 · Mapa de registros de la **Guía ESP32 + HMI v2**

Sirve para validar el tramo **ESP32 → RS485 → PSoC real** antes de integrar el PID. La PSoC responde como esclavo 1 con el mapa v2 completo, simula la planta y escribe por el USB del KitProg todo lo que recibe y responde.

## Archivos

| Archivo | Qué hace |
|---|---|
| `main.c` | Recepción RS485, detección de fin de trama, respuesta, log, bucle principal |
| `modbus_slave.c/.h` | Lógica Modbus pura (FC 01, 03, 04, 05, 06, 16), sin hardware: **reutilizable por el frente de control** |
| `plant_sim.c/.h` | Planta simulada (altura, PWM, error, estado). Es lo que se reemplaza por el PID real |
| `debug_log.c/.h` | Salida legible con unidades por `UART_DBG` |
| `consola.c/.h` | Comandos desde el monitor serie |

---

## 1. Crear el proyecto

1. **File → New → Project…** → *Target hardware*: **CY8CKIT-059 (PSoC 5LP)** → *Empty schematic*. Nombre sugerido: `Levitador_ModbusTest`.
2. En el *Workspace Explorer*: clic derecho sobre **Source Files → Add → Existing Item…** y agregue los `.c`. Haga lo mismo en **Header Files** con los `.h`.
3. Reemplace el `main.c` que crea el proyecto por el de esta carpeta (o pegue su contenido encima).

## 2. TopDesign.cysch

Coloque estos componentes (Component Catalog):

### 2.1 `UART_RS485`: Communications → UART [v2.50]

| Pestaña | Parámetro | Valor |
|---|---|---|
| Configure | Mode | **Full UART (TX + RX)** |
| | Bits per second | **19200** |
| | Data bits / Parity / Stop | **8 / None / 1** |
| | Flow control | None |
| Advanced | Clock selection | Internal clock |
| | **Hardware TX Enable** | **✔ activado** (aparece el terminal `tx_en`) |
| | RX buffer size | **128** |
| | TX buffer size | **64** |
| | Interrupt sources | RX – On Byte Received (queda activo al usar buffer > 4) |
| | Oversampling rate | 8x (por defecto) |

Conexiones (Digital Port → *Digital Input Pin* / *Digital Output Pin*):

| Terminal UART | Pin (nombre) | Tipo / Drive mode | Pin físico sugerido | MAX485 #2 |
|---|---|---|---|---|
| `rx` | `Pin_RS485_RX` | Digital Input, **Resistive Pull Up**, *Initial drive state* = **High (1)** | **P3[0]** | **RO** |
| `tx` | `Pin_RS485_TX` | Digital Output, Strong Drive | **P3[1]** | **DI** |
| `tx_en` | `Pin_RS485_DE` | Digital Output, Strong Drive | **P3[2]** | **DE + RE (unidos)** |

> **El pull-up en RX es obligatorio.** Mientras la PSoC transmite, `tx_en` = 1 deshabilita el receptor del MAX485 y **RO queda en alta impedancia**. Sin pull-up la UART lee basura justo después de cada respuesta. En PSoC 5LP el pull-up resistivo solo actúa si el *Initial drive state* es **High**.

### 2.2 `UART_DBG`: Communications → UART [v2.50]

| Parámetro | Valor |
|---|---|
| Mode | Full UART (TX + RX) |
| Bits per second | **115200**, 8N1 |
| RX buffer size | **64** |
| TX buffer size | **512** (para que el log no bloquee) |

| Terminal | Pin | Pin físico |
|---|---|---|
| `rx` | `Pin_DBG_RX` (Digital Input, HiZ) | **P12[6]** (viene del KitProg) |
| `tx` | `Pin_DBG_TX` (Digital Output, Strong) | **P12[7]** (va al KitProg) |

### 2.3 `Pin_LED` (opcional)

Digital Output Pin, **desmarque *HW connection***, nombre `Pin_LED`, pin **P2[1]** (LED azul de la placa). Parpadea con cada respuesta enviada.
Si no lo pone, cambie `#define USAR_LED 1u` a `0u` en `main.c`.

## 3. Design Wide Resources (`.cydwr`)

- **Pins:** asigne los pines de las tablas anteriores.
- **System:**
  - **Heap Size:** `0x400` (lo usan `snprintf` / `vsnprintf` de newlib-nano).
  - **Stack Size:** `0x1000`.
  - **VDDD, VDDA y VDDIO0..3: 5.0 V** (la 059 alimentada por USB trabaja a 5 V; así el MAX485 de 5 V se conecta directo).
- **Clocks:** con el reloj por defecto de la 059 (BUS_CLK 24 MHz) los dos UART quedan con error de baudios < 1 %. Si PSoC Creator muestra una advertencia de precisión, súbalo a 48 MHz.

Luego: **Build → Build Levitador_ModbusTest** y **Debug → Program** (con el KitProg conectado por USB).

## 4. Cableado

```
ESP32 ── MAX485 #1 ══ A/B (par trenzado) ══ MAX485 #2 ── PSoC 5LP (CY8CKIT-059)
                     GND ─────────────────── GND
MAX485 #2:  VCC → VDD (5 V) de la 059    GND → GND de la 059
            RO  → P3[0]   DI → P3[1]     DE+RE → P3[2]
            A ↔ A del #1  B ↔ B del #1   120 Ω entre A y B en cada extremo
```

**El GND común entre las dos placas no es opcional** (ver la sección 2.3 de la guía).

## 5. Monitor serie

- Puerto: **KitProg USB-UART (COMx)**, 115200 8N1.
- PuTTY / Tera Term: sin *local echo* (la PSoC hace el eco). Monitor de Arduino IDE: fin de línea **Newline** o **Both NL & CR**.
- Abra a la vez el puerto de depuración de la ESP32 (GPIO25) para comparar las dos puntas: `[TEL]` de la ESP32 contra `[RD]` de la PSoC.

## 6. Procedimiento de prueba

### Etapa 0: sniffer (valida la capa física)

Escriba `etapa 0`. La PSoC solo escucha:

```
[SNIF] 8 B | CRC OK | id=1 FC04 dir=0x0000 dato=7
       01 04 00 00 00 07 B1 C8
```

- La ESP32 marcará `codigo 0xE2` (timeout) cada 2 s: **es lo esperado**, porque nadie responde.
- Si sale `CRC MALO`, bytes extraños o nada, el problema es físico: A/B invertidos, falta el GND común, baudios distintos o falta el pull-up.

### Etapa 1: esclavo Modbus

Escriba `etapa 1` (es la etapa por defecto al arrancar):

```
[LINK] Recibiendo tramas validas del maestro
[RX] FC06 HR0 Setpoint = 30.0 cm [crudo=300] -> respondido OK
[RX] FC05 Coil0 Habilitar control = ON -> respondido OK
[RD] 1 s: FC04 x10 | ok=1520 crc=2 | SP=30.0cm Alt=22.6cm PWM=69.6% Err=7.4cm Est=0b000001
```

Verifique:

1. En la ESP32 desaparecen los `0xE2` y el campo `Modbus=` de `[TEL]` baja a unos 10–20 ms.
2. `FC04 x10` por segundo, que corresponde al sondeo de 100 ms.
3. Cambie el SP en LabVIEW: aparece `[RX] FC06 HR0 …` y el `sp` de la línea DATA cambia.
4. Con `EN:1` y modo auto, la altura simulada sube hacia el setpoint y la gráfica del HMI la sigue.
5. Prueba `MODE:0` + `PWM:700`, cambio de ganancias, `EMG:1` (la altura cae) y `RST:1` (en `dump`, *pulsos RST* aumenta).

### Etapa 2: inyectar valores desde la consola

| Comando | Para probar |
|---|---|
| `alt 226` | Altura fija en 22.6 cm |
| `err -35` | Que el HMI maneje error **negativo** (pendiente de la sección 7) |
| `est 0x21` | Indicador de *fuera de rango* (bit 5) en el HMI |
| `est 0x11` | Indicador de *timeout de comunicación* (bit 4) |
| `mute 1` | **Enlace caído**: la PSoC deja de responder, la ESP32 da `0xE2` y el HMI deja de recibir DATA |
| `sim` | Vuelve a la planta simulada |
| `log 2` | Todo en hex (cada RX y TX); útil si algo no cuadra |
| `dump` | Estado completo de los registros |
| `reset` | Contadores de tramas y CRC en cero (antes de la prueba de 30 min) |

### Prueba de resistencia

`reset` y luego 30 minutos de operación. En el resumen `[RD]`, **`crc=` no debe crecer**.

## 7. Detalles de implementación

- **Fin de trama:** silencio ≥ 3.5 caracteres (2.0 ms a 19200). Se mide con SysTick a 1 ms (umbral de 3 ticks, es decir, de 2 a 3 ms reales).
- **Orden:** primero se responde por RS485 y después se escribe el log. Ningún `printf` va dentro de una interrupción.
- **DE/RE:** lo maneja el hardware del UART (`tx_en`); se desactiva solo después del bit de parada del último byte.
- **Excepciones:** dirección fuera del mapa → `0x02`; valor fuera de rango (p. ej. SP > 1000, Modo > 1) → `0x03`; función no soportada → `0x01`.
- **CRC malo o trama para otro esclavo:** no se responde (como manda Modbus). El CRC malo se cuenta en IR6.
- **Broadcast (id 0):** se ejecutan las escrituras y no se responde.
- **Bit 4 del estado (timeout):** se activa si pasa más de 1 s sin tramas válidas.

## 8. Para el frente de control

Para pasar al firmware real: conservar `modbus_slave.c/.h` y la recepción de `main.c`, y reemplazar `plant_step()` por el lazo PID real, que escribe `ir[IR_ALTURA]`, `ir[IR_PWM]`, `ir[IR_ERROR]` e `ir[IR_ESTADO]` y lee `hr[]` y `coil[]`. El log y la consola se pueden dejar con `log 0` o quitarse.
