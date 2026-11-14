# Levitador neumático — PID en PSoC 5LP, Modbus RTU/RS485, ESP32 y HMI en LabVIEW

Proyecto final · Ingeniería Electrónica · UCEVA · 2026-II

Sistema que mantiene una pelota liviana a una altura elegida por el usuario dentro de un tubo de 42 cm, regulando el caudal de un ventilador con un controlador **PID**. El control corre en una **PSoC 5LP** (esclavo Modbus); una **ESP32** actúa como **maestro Modbus RTU** sobre **RS485** y hace de puente con un **HMI en LabVIEW**, desde el que se supervisa y opera la planta.

La arquitectura reproduce la de un sistema de automatización industrial: un controlador dedicado que se comunica por un protocolo industrial con un supervisor encargado del monitoreo y la interacción con el operador (enunciado completo en [`Recursos/01_Enunciado/proyecto_final_2026_II.pdf`](Recursos/01_Enunciado/proyecto_final_2026_II.pdf)).

---

## Contenido

1. [Implementación final (qué código se usa)](#1-implementación-final-qué-código-se-usa)
2. [Arquitectura](#2-arquitectura)
3. [Cómo se conectan los componentes](#3-cómo-se-conectan-los-componentes)
4. [El lazo de control](#4-el-lazo-de-control)
5. [Estructura del repositorio](#5-estructura-del-repositorio)
6. [Hardware](#6-hardware)
7. [Software necesario](#7-software-necesario)
8. [Reproducir el proyecto paso a paso](#8-reproducir-el-proyecto-paso-a-paso)
9. [Pruebas y validación](#9-pruebas-y-validación)
10. [Evolución del desarrollo](#10-evolución-del-desarrollo)
11. [Documentación de referencia](#11-documentación-de-referencia)
12. [Estado y pendientes](#12-estado-y-pendientes)
13. [Equipo](#13-equipo)

---

## 1. Implementación final (qué código se usa)

El repositorio guarda también versiones intermedias y pruebas. **Lo que corre en el montaje final son estos tres programas:**

| Componente | Carpeta | Archivo principal | Herramienta | Documentación |
|---|---|---|---|---|
| ★ **Controlador: esclavo Modbus + PID** (PSoC 5LP, CY8CKIT-059) | [`Codigo/PSoC/psoc_levitador_pid/`](Codigo/PSoC/psoc_levitador_pid) | `psoc_levitador_pid.cywrk` → `Levitador.cydsn/main.c` | PSoC Creator 4.4 | [README](Codigo/PSoC/psoc_levitador_pid/README.md) |
| ★ **Pasarela: maestro Modbus v1.1** (ESP32) | [`Codigo/Firmware esp32/levitador_esp32_maestro-v1.1/`](Codigo/Firmware%20esp32/levitador_esp32_maestro-v1.1) | `levitador_esp32_maestro-v1.1.ino` + `src/ModbusMaster/` | Arduino IDE + core ESP32 | [README](Codigo/Firmware%20esp32/README.md) |
| ★ **Supervisión: HMI** (PC) | [`HMI LabVIEW/LEVITADOR NEUMATICO V7/`](HMI%20LabVIEW/LEVITADOR%20NEUMATICO%20V7) | `Levitador_Neumatico.lvproj` → `Main.vi` | LabVIEW 2026 + NI-VISA | Sección 3.1 de este documento y la guía v4 |

El **contrato de comunicación** entre los tres (protocolo de texto, mapa de registros, escalado, tiempos) está en [`Recursos/03_Guia_Integracion/Guia ESP32 HMI Modbus v4.pdf`](Recursos/03_Guia_Integracion/Guia%20ESP32%20HMI%20Modbus%20v4.pdf), que es la **fuente de verdad**. El cableado completo está en [`Recursos/04_Referencias/CONEXIONES.md`](Recursos/04_Referencias/CONEXIONES.md) y todos los comandos de consola en [`Recursos/04_Referencias/COMANDOS.md`](Recursos/04_Referencias/COMANDOS.md).

Programas de apoyo que no corren en el montaje final pero forman parte del resultado:

| Programa | Carpeta | Para qué se usó |
|---|---|---|
| PID autónomo en la ESP32 | [`Codigo/PID/levitador_esp32_PID/`](Codigo/PID/levitador_esp32_PID) | Sintonizar el PID directamente sobre la planta. El PID de la PSoC es una transcripción línea por línea de este `.ino` |
| Firmware de prueba Modbus (PSoC) | [`Codigo/PSoC/Levitador_ModbusTest/`](Codigo/PSoC/Levitador_ModbusTest) | Validar el tramo ESP32 → RS485 → PSoC con planta simulada antes de integrar el PID; sirve como sniffer del bus |
| Modelo y diseño preliminar (Python) | [`Codigo/PID/`](Codigo/PID) | Modelo físico, respuesta al escalón en lazo abierto y ganancias de partida por colocación de polos |
| Pruebas del firmware en el PC | [`Codigo/PSoC/psoc_levitador_pid/pruebas/`](Codigo/PSoC/psoc_levitador_pid/pruebas) | Equivalencia del PID con el `.ino` y simulación del firmware completo (§9) |

---

## 2. Arquitectura

```
┌──────────────┐  USB serie 115200 8N1   ┌──────────────┐  UART2  ┌──────────┐  RS485 19200 8N1  ┌──────────┐  UART  ┌────────────────────┐
│  PC          │  texto CLAVE:VALOR  ──► │  ESP32       │ ──────► │ MAX485   │ ═══ A/B + GND ═══ │ MAX485   │ ─────► │  PSoC 5LP          │
│  HMI LabVIEW │ ◄── DATA,... (2 Hz)     │  maestro     │ ◄────── │   #1     │    Modbus RTU     │   #2     │ ◄───── │  esclavo id 1      │
└──────────────┘                         │  Modbus      │         └──────────┘                   └──────────┘        │  PID 25 Hz         │
                                         └──────┬───────┘                                                          └──┬──────────┬──────┘
                                                │ UART1 GPIO25                                    KitProg USB (consola)  │          │
                                                ▼                                                 115200 ◄───────────────┤          │
                                         depuración (USB-TTL)                                                            │          │
                                                                                              HC-SR04 (TRIG/ECHO) ───────┘          │
                                                                                              PWM 25 kHz → IRLZ44N → ventilador 24 V┘
```

| Bloque | Responsabilidad |
|---|---|
| **HMI (LabVIEW)** | Pantalla del operador: setpoint, ganancias, modo manual/automático, habilitar, paro de emergencia, reset del integrador, gráfica altura vs. setpoint y estado del enlace |
| **ESP32 (maestro)** | Traduce cada comando de texto a una escritura Modbus; sondea la telemetría de la PSoC cada 500 ms y la devuelve a LabVIEW en una línea `DATA,...` con el estado del enlace. No hace control |
| **PSoC 5LP (esclavo)** | Mide la altura (HC-SR04 por interrupción), ejecuta el PID + feedforward cada 40 ms, genera el PWM del ventilador, atiende el Modbus y aplica las protecciones (timeout de comunicación, paro por sensor, emergencia) |
| **Planta** | Tubo de 42 cm, pelota de 5.5 cm (icopor + cartones), ventilador YXD9225HB24R de 24 V a 7 cm del piso y sensor HC-SR04 arriba del tubo apuntando hacia abajo |

---

## 3. Cómo se conectan los componentes

### 3.1 HMI ↔ ESP32: protocolo de texto (USB, 115200 8N1)

**Comandos (LabVIEW → ESP32):** una línea `CLAVE:VALOR\n` por comando, con el valor entero ya escalado.

| Comando | Significado | Se escribe en (Modbus) |
|---|---|---|
| `SP:150` | Setpoint 15.0 cm (cm × 10; máximo útil ≈ 310) | Holding 0x0000 (FC06) |
| `KP:100` · `KI:60` · `KD:5` | Ganancias × 100 (arranque 1.00 / 0.60 / 0.05) | Holding 0x0001–0x0003 |
| `MODE:1` | 1 = automático (PID), 0 = manual | Holding 0x0004 |
| `PWM:500` | Duty manual 50.0 % (por mil) | Holding 0x0005 |
| `EN:1` | Habilitar control | Coil 0x0000 (FC05) |
| `RST:1` | Reiniciar el integrador (pulso de 50 ms) | Coil 0x0001 |
| `EMG:1` | Paro de emergencia | Coil 0x0002 |

**Telemetría (ESP32 → LabVIEW), cada 500 ms y siempre (falle o no la lectura):**

```
DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc,enlace\n
DATA,150,154,543,-4,3,1520,0,Conexion OK
```

`sp`, `altura` y `error` en cm × 10 (`error` = sp − altura, con signo), `pwm` por mil, `estado` es la palabra de bits de la §4.3, y `enlace` es un texto (`Conexion OK`, `Conexion inestable (...)`, `Sin conexion (...)`, `Esclavo rechaza peticion (...)`, ...). Si la lectura Modbus falla, los campos numéricos repiten los últimos valores válidos y `enlace` indica el problema.

### 3.2 ESP32 ↔ PSoC: Modbus RTU sobre RS485

Esclavo **id 1**, **19200 8N1**, funciones 01, 03, 04, 05, 06 y 16. Escalado: alturas en cm × 10, ganancias × 100 (% de duty por cm), duty en por mil.

| Tipo | Direcciones | Contenido |
|---|---|---|
| Holding registers | 0x0000–0x0005 | Setpoint, Kp, Ki, Kd, Modo, PWM manual |
| Input registers | 0x0000–0x0006 | Setpoint vigente (copia de HR0), altura, PWM aplicado, error, estado, tramas válidas, errores de CRC |
| Coils | 0x0000–0x0002 | Habilitar, reset del integrador, emergencia |

La ESP32 lee los 7 input registers en **una sola transacción FC04** (`01 04 00 00 00 07 B1 C8`), en el mismo orden que la línea `DATA`. Así, cada línea que ve el HMI corresponde a una única lectura coherente de la PSoC.

### 3.3 PSoC ↔ planta

| Señal | Pin PSoC | Detalle |
|---|---|---|
| TRIG del HC-SR04 | P2[4] | Pulso de 10 µs cada 40 ms |
| ECHO del HC-SR04 | P2[5] | Interrupción en ambos flancos + Timer de 1 MHz (directo, el puerto está a 5 V) |
| PWM del ventilador | P2[3] | 25 kHz, resolución 0.1 %, → 220 Ω → gate del IRLZ44N (lado bajo) |
| RS485 (MAX485 #2) | P3[0] RX (pull-up) · P3[1] TX · P3[2] DE/RE | UART con *Hardware TX Enable* |
| Consola / log | P12[6..7] | KitProg USB-UART, 115200 |

### 3.4 Tiempos del sistema

| Parámetro | Valor | Dónde |
|---|---|---|
| Periodo del PID | 40 ms (25 Hz) | PSoC, `control.h` |
| Periodo de sondeo / telemetría | 500 ms (2 Hz) | ESP32 |
| Timeout de respuesta del maestro | 100 ms (copia local de ModbusMaster; la librería trae 2000 ms) | ESP32, `src/ModbusMaster/ModbusMaster.h` |
| Respuesta de la PSoC | ≈ 3 ms (contrato: < 100 ms) | Medido en la simulación del firmware |
| Enlace declarado caído (ESP32) | 3 fallos seguidos (1–1.5 s) | ESP32 |
| Timeout de seguridad (PSoC) | 1.5 s sin tramas válidas → EN = 0, ventilador apagado | PSoC, `main.c` |
| Paro por sensor (PSoC) | 20 lecturas sin eco (≈ 0.8 s) con el lazo cerrado → EN = 0 | PSoC, `control.c` |

Tras un timeout de seguridad o un paro por sensor la PSoC **no arranca sola**: el operador debe enviar `EN:1`.

---

## 4. El lazo de control

### 4.1 Estructura

```
u = uFF + Kp·e + I − Kd·v_filtrada        e = sp − altura
```

- **Feedforward `uFF`**: duty con el que la pelota flota; el PID solo corrige alrededor de ese punto.
- **Derivada sobre la medida** (velocidad filtrada, Tf = 0.12 s), sin patada derivativa ante cambios de setpoint.
- **Anti-windup** por integración condicional e integrador limitado a ± 40 %.
- **Piso de duty** en vuelo (evita que la pelota caiga a plomo con errores negativos grandes).
- **Medición**: mediana de 3 lecturas, rechazo de saltos > 10 cm (ecos en la pared) y calibración automática del fondo al arrancar (≈ 1.1 s, pelota abajo).
- **Manual → automático sin salto**: el integrador arranca en `duty manual − uFF`.

### 4.2 Parámetros finales (ajustados en placa)

| Kp | Ki | Kd | uFF | Piso | Setpoint máximo útil |
|---|---|---|---|---|---|
| 1.00 | 0.60 | 0.05 | 49 % | 20 % | fondo − 5 cm (≈ 31–32 cm) |

Kp, Ki y Kd se cambian en vivo desde el HMI; `uFF` y el piso no están en el mapa Modbus y solo se cambian por la consola de la PSoC (`ff 49`, `fl 20`). Los valores de arranque están en `control.h` (`CTRL_*_DEFECTO`).

### 4.3 Palabra de estado (IR 0x0004)

| Bit | Significado |
|---|---|
| 0 | Sensor OK (menos de 3 lecturas fallidas seguidas) |
| 1 | Pelota en el objetivo (\|error\| ≤ 5 mm, solo con PID) |
| 2 | Salida del PID saturada |
| 3 | Integrador en su límite (± 40 %) |
| 4 | Más de 1.5 s sin tramas válidas del maestro |
| 5 | Pelota más de 2 cm por encima del setpoint máximo |

---

## 5. Estructura del repositorio

★ = usado en la implementación final.

```
Proyecto_LN/
├── README.md                                   <- este documento
├── Codigo/
│   ├── PSoC/
│   │   ├── psoc_levitador_pid/              ★  Firmware final de la PSoC: esclavo Modbus + PID
│   │   │   ├── psoc_levitador_pid.cywrk           Workspace de PSoC Creator (abrir este)
│   │   │   ├── Levitador.cydsn/                   Proyecto: fuentes .c/.h, TopDesign, pines (.cydwr)
│   │   │   ├── pruebas/                           Pruebas en el PC (gcc): equivalencia y simulación
│   │   │   └── README.md
│   │   └── Levitador_ModbusTest/                  Firmware de prueba: esclavo Modbus + planta simulada
│   │       ├── Levitador_ModbusTest.cywrk
│   │       ├── Levitador.cydsn/
│   │       └── README.md
│   ├── Firmware esp32/
│   │   ├── levitador_esp32_maestro-v1.1/    ★  Firmware final de la ESP32 (maestro Modbus v1.1)
│   │   │   └── src/ModbusMaster/                  Copia local de ModbusMaster con timeout de 100 ms
│   │   ├── levitador_esp32_maestro/               v1.0 (histórico)
│   │   ├── Pruebas/                               Sketches de desarrollo (V1–V9, prueba con LabVIEW, prueba PWM)
│   │   └── README.md
│   └── PID/
│       ├── levitador_esp32_PID/                   PID autónomo en la ESP32 (sintonía en planta real)
│       ├── modelo_levitador.py                    Modelo físico no lineal y linealización
│       ├── escalon_lazo_abierto.py                Respuesta al escalón en lazo abierto
│       └── pid_preliminar.py                      Ganancias de partida por colocación de polos
├── HMI LabVIEW/
│   ├── LEVITADOR NEUMATICO V7/              ★  Versión final del HMI
│   └── LEVITADOR NEUMATICO, V2 … V6/              Versiones anteriores
└── Recursos/                                      Documentación de apoyo (§11)
    ├── 01_Enunciado/                              Enunciado del proyecto
    ├── 02_Diseno_Control/                         Informes del modelo, el escalón y el PID preliminar
    ├── 03_Guia_Integracion/                       Guía ESP32 + HMI v4 (vigente) y Versiones_anteriores/
    └── 04_Referencias/                            CONEXIONES.md, COMANDOS.md y guía de Modbus RTU
```

Notas sobre los proyectos:

- **PSoC Creator** regenera `codegentemp/`, `CortexM3/` y `Generated_Source/` al compilar. El `.hex` listo para programar queda en `Levitador.cydsn/CortexM3/ARM_GCC_541/Debug/Levitador.hex`.
- **LabVIEW**: en cada versión el VI principal está en `.Levitador_Neumatico.UserState/Main.vi` (carpeta con punto inicial, que en algunos exploradores aparece oculta). El `.lvproj` lo referencia ahí, así que hay que abrir el proyecto y no mover el VI por fuera de LabVIEW.
- **Arduino**: cada sketch está en una carpeta con su mismo nombre, como exige el IDE.

---

## 6. Hardware

| Elemento | Cantidad | Nota |
|---|---|---|
| CY8CKIT-059 (PSoC 5LP, CY8C5888LTI-LP097) | 1 | Alimentada por USB del KitProg; VDDIO a 5 V |
| ESP32 (placa de desarrollo) | 1 | Maestro Modbus |
| Módulo MAX485 | 2 | #1 en la ESP32, #2 en la PSoC; 120 Ω entre A y B en cada extremo |
| Sensor ultrasónico HC-SR04 | 1 | Arriba del tubo, apuntando hacia abajo |
| MOSFET IRLZ44N + resistencia de 220 Ω | 1 | Conmutación del ventilador en el lado bajo |
| Ventilador YXD9225HB24R (24 V) | 1 | Con diodo de rueda libre en antiparalelo |
| Fuente de 24 V | 1 | GND unido al punto de tierra común |
| Tubo de 42 cm y pelota de 5.5 cm (icopor + cartones) | 1 | Planta |
| Cable UTP (un par trenzado para A/B + un hilo para GND) | 1 | Bus RS485 |
| Adaptador USB-TTL | 1 | Puerto de depuración de la ESP32 (GPIO25) |
| Adaptador USB-RS485 | opcional | Para probar la ESP32 sin la PSoC, contra un esclavo simulado en el PC |

Pines resumidos:

| Placa | Pines |
|---|---|
| ESP32 | MAX485 #1: DI → GPIO17 (TX2), RO → GPIO16 (RX2), DE+RE → GPIO4 · depuración TX → GPIO25 |
| PSoC | MAX485 #2: RO → P3[0], DI → P3[1], DE+RE → P3[2] · PWM P2[3] · TRIG P2[4] · ECHO P2[5] · LED P2[1] |

> **Tierra común obligatoria** entre todos los bloques (incluido un hilo de GND en el bus RS485), unida en un solo punto para que la corriente del ventilador no pase por los GND de la lógica. El detalle, con el checklist antes de energizar, está en [`Recursos/04_Referencias/CONEXIONES.md`](Recursos/04_Referencias/CONEXIONES.md).

> **GPIO25 de la ESP32** es el TX de depuración en el firmware maestro y la salida PWM en el PID autónomo: revisar qué está conectado antes de cambiar de firmware.

---

## 7. Software necesario

| Herramienta | Versión | Para |
|---|---|---|
| PSoC Creator | 4.4 | Compilar y programar la PSoC (incluye el compilador ARM GCC 5.4.1) |
| Arduino IDE + core ESP32 de Espressif | core 2.x o 3.x | Firmware de la ESP32. ModbusMaster v2.0.1 ya va incluida en el sketch final |
| LabVIEW + NI-VISA | LabVIEW 2026 (proyecto guardado en la versión 26.0) | HMI |
| Monitor serie (PuTTY, Tera Term o el de Arduino IDE) | — | Consola de la PSoC (KitProg) y depuración de la ESP32 |
| Python 3 + numpy + matplotlib | — | Scripts de diseño y gráfica de las pruebas |
| gcc (Linux o WSL) | — | Pruebas del firmware de la PSoC en el PC |

---

## 8. Reproducir el proyecto paso a paso

El orden sigue la regla de la guía: **validar cada enlace por separado** antes de unir todo.

1. **Armar la planta y el cableado** según [`Recursos/04_Referencias/CONEXIONES.md`](Recursos/04_Referencias/CONEXIONES.md) y revisar su checklist (sección 8).
2. **PSoC**: abrir `Codigo/PSoC/psoc_levitador_pid/psoc_levitador_pid.cywrk` en PSoC Creator 4.4 → *Build* → *Program* con el KitProg por USB. Abrir el puerto *KitProg USB-UART* a 115200: debe aparecer el encabezado del firmware y `[CTRL] Calibracion OK: base = …` (con la pelota abajo y el ventilador apagado).
3. **ESP32**: abrir `Codigo/Firmware esp32/levitador_esp32_maestro-v1.1/levitador_esp32_maestro-v1.1.ino` en Arduino IDE, elegir la placa ESP32 y cargar. En el puerto de depuración (GPIO25, 115200) deben salir líneas `[TEL] ...` y `[ENLACE] Conexion OK`. Detalles de compilación en su [README](Codigo/Firmware%20esp32/README.md#compilación).
4. **HMI**: abrir `HMI LabVIEW/LEVITADOR NEUMATICO V7/Levitador_Neumatico.lvproj`, abrir `Main.vi`, seleccionar el puerto COM de la ESP32 (115200 8N1) y ejecutar. El indicador de enlace debe mostrar `Conexion OK`.
5. **Prueba de la cadena sin ventilador**: en la consola de la PSoC, `fuente sim`; desde el HMI enviar un setpoint y `EN:1`. La gráfica debe seguir a la planta simulada. Volver con `fuente real`.
6. **Puesta en marcha con la planta real**: setpoint dentro del rango útil (≤ 31 cm), modo automático y `EN:1`. Con las ganancias de arranque la pelota sube y se sostiene; se pueden ajustar Kp, Ki y Kd en vivo.
7. **Validación completa**: procedimiento P0–P6 del [README de la PSoC](Codigo/PSoC/psoc_levitador_pid/README.md#6-procedimiento-de-prueba-en-placa) (sensor, lazo abierto, PID por consola, PID desde el HMI, seguridad y resistencia de 30 min).

Si algo falla: `fuente sim` en la PSoC separa en segundos un problema de comunicación de uno de la planta; el campo `enlace` del HMI dice si el problema está entre la ESP32 y la PSoC, y el modo sniffer (`etapa 0`) de la PSoC muestra lo que llega al bus.

---

## 9. Pruebas y validación

| Prueba | Dónde | Cómo correrla | Resultado |
|---|---|---|---|
| Equivalencia del PID de la PSoC con el `.ino` de la ESP32 | `Codigo/PSoC/psoc_levitador_pid/pruebas/equivalencia.c` | `./correr.sh` | Diferencia máxima de duty 0.0 % en 3000 pasos |
| Firmware completo de la PSoC simulado (HC-SR04 a nivel de pines, planta, ESP32 sondeando) | `.../pruebas/sim_host.c` | `./correr.sh` | 28/28 verificaciones; respuesta Modbus ≤ 3 ms |
| ESP32 contra esclavo simulado (sin PSoC) | Adaptador USB-RS485 + simulador de esclavo en el PC | [README de la ESP32](Codigo/Firmware%20esp32/README.md#pruebas-sin-la-psoc) | Comandos, telemetría y estados del enlace |
| Bus RS485 y esclavo con planta simulada | `Codigo/PSoC/Levitador_ModbusTest/` | [Su README](Codigo/PSoC/Levitador_ModbusTest/README.md#6-procedimiento-de-prueba) | Sniffer, respuestas, valores forzados y enlace caído |
| Integración en placa | Montaje completo | P0–P6 del README de la PSoC | Ver §12 |

La planta simulada sirve para verificar la lógica, no para juzgar la sintonía: la sintonía se validó sobre la planta real.

---

## 10. Evolución del desarrollo

| Etapa | Qué se hizo | Archivos |
|---|---|---|
| Diseño del control | Modelo físico, identificación por escalón en lazo abierto y PID preliminar por colocación de polos (con parámetros estimados, antes de tener la planta) | `Codigo/PID/*.py` y `Recursos/02_Diseno_Control/` |
| Actuador | Primera prueba del PWM de 25 kHz al ventilador | `Codigo/Firmware esp32/Pruebas/prueba_pwm_version1/` |
| Maestro Modbus | Versiones V1–V9 contra un esclavo simulado en el PC: parser de comandos, validación de rangos, puerto de depuración, prefijo `DATA,`, setpoint en IR 0, parser tolerante | `Codigo/Firmware esp32/Pruebas/` |
| Pasarela v1.0 → v1.1 | v1.0: primera versión real. v1.1: campo `enlace`, trama siempre enviada, timeout de 100 ms, sondeo de 500 ms | `Codigo/Firmware esp32/levitador_esp32_maestro*/` |
| HMI | Versiones V1–V7 del VI | `HMI LabVIEW/` |
| Sintonía | PID + feedforward sintonizado en la planta real con la ESP32 autónoma | `Codigo/PID/levitador_esp32_PID/` |
| Esclavo en la PSoC | Firmware de prueba (Modbus + planta simulada) y luego el firmware final con el PID portado y ajustado en placa | `Codigo/PSoC/` |

---

## 11. Documentación de referencia

| Carpeta (`Recursos/`) | Archivo | Contenido |
|---|---|---|
| `01_Enunciado/` | `proyecto_final_2026_II.pdf` | Enunciado del proyecto |
| `02_Diseno_Control/` | `Modelo_Fisico_Levitador.docx` | Modelo físico de la planta (base de `modelo_levitador.py`) |
| | `Respuesta_Escalon_Lazo_Abierto.docx` | Análisis del escalón en lazo abierto (base de `escalon_lazo_abierto.py`) |
| | `Diseno_PID_Preliminar.docx` | Cálculo de las ganancias de partida (base de `pid_preliminar.py`) |
| `03_Guia_Integracion/` | `Guia ESP32 HMI Modbus v4.pdf` | **Vigente.** Contrato de comunicación, mapa de registros, comportamiento de la PSoC, plan de trabajo y acuerdos del equipo |
| `03_Guia_Integracion/Versiones_anteriores/` | `Guia ESP32 HMI Modbus v1.pdf` … `v3.pdf` | Versiones anteriores de la guía (histórico) |
| `04_Referencias/` | `CONEXIONES.md` | Referencia unificada de conexiones físicas, tierras y checklist |
| | `COMANDOS.md` | Referencia unificada de comandos de todos los firmwares |
| | `Especificación Modbus RTU.pdf` | Guía rápida del protocolo Modbus RTU |

La guía v4 menciona `CONEXIONES.md` y `COMANDOS.md` por nombre: ambos están en `04_Referencias/`.

---

## 12. Estado y pendientes

Según el plan de trabajo de la guía v4 (Sección 6):

- [x] ESP32 maestro v1.1 implementado y en uso con la PSoC.
- [x] PSoC con el mapa Modbus completo, PID desplegado y ganancias ajustadas en placa.
- [x] MAX485 #2 en la PSoC, telemetría real y setpoint desde el HMI moviendo la pelota.
- [ ] Desde el HMI: manual ↔ auto, ganancias en vivo, `RST`, `EMG` (por confirmar con el VI final).
- [ ] Pruebas de seguridad (desconectar A/B y ECHO) y de resistencia de 30 min.
- [ ] Evidencia de la integración (video o capturas con la pelota real).

---

## 13. Equipo

En orden alfabético por primer apellido:

- Juan Manuel Cardenas Velasquez
- Arlyn Cardozo Pino
- Jose David España Chalaca
- Alejandro González Rico
- Juan David Serrato Vanegas
- Dairom Triviño Espinosa
