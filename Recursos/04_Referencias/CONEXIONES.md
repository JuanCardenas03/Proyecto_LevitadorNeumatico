# Levitador neumático – Referencia unificada de conexiones físicas

Todas las conexiones del proyecto, separadas por firmware/placa. Las tomé de los README, los `.ino`, `hw_levitador.c` y la **Guía ESP32 + HMI v4** (secciones 2 y 7).

## 0. Vista general del sistema

```
                USB 115200 (texto)            UART2 TTL 19200            RS485 19200 Modbus RTU            TTL
 PC / LabVIEW ───────────────────── ESP32 maestro ─────────── MAX485 #1 ══ A/B + GND ══ MAX485 #2 ─────────── PSoC 5LP (esclavo 1)
                                       │                                                                         │   │   │
                                       └─ UART1 GPIO25 → USB-TTL (depuración)          KitProg USB (depuración) ─┘   │   │
                                                                                                    HC-SR04 ─────────┘   │
                                                                                     Pin_PWM → IRLZ44N → ventilador 24 V ┘
```

### Puertos serie del sistema

| Puerto | Placa | Pines | Velocidad | Uso |
|---|---|---|---|---|
| USB (UART0) | ESP32 maestro | USB de la placa | 115200 8N1 | Protocolo de texto con LabVIEW (solo `CLAVE:VALOR` y `DATA,...`) |
| UART1 | ESP32 maestro | TX GPIO25 / RX GPIO26 (sin uso) | 115200 8N1 | Depuración humana, por adaptador USB-TTL |
| UART2 | ESP32 maestro | TX GPIO17 / RX GPIO16 / DE+RE GPIO4 | 19200 8N1 | Bus RS485, maestro Modbus |
| `UART_RS485` | PSoC 5LP | RX P3[0] / TX P3[1] / `tx_en` P3[2] | 19200 8N1 | Bus RS485, esclavo Modbus id 1 |
| `UART_DBG` | PSoC 5LP | RX P12[6] / TX P12[7] (internos al KitProg) | 115200 8N1 | Consola y log por el USB del KitProg |
| USB | ESP32 PID | USB de la placa | 115200 | Monitor serie / Serial Plotter |

---

## 1. PSoC 5LP – esclavo Modbus + PID (`Codigo/PSoC/psoc_levitador_pid`, firmware final)

Placa: **CY8CKIT-059**. En el `.cydwr`, **VDDD, VDDA y VDDIO0..3 = 5.0 V** (alimentada por USB).

### 1.1 Asignación de pines

| Pin lógico (TopDesign) | Pin físico | Tipo / drive | Va a |
|---|---|---|---|
| `Pin_RS485_RX` (`rx` de `UART_RS485`) | **P3[0]** | Digital Input, **Resistive Pull Up**, *Initial drive state* = **High** | RO del MAX485 #2 |
| `Pin_RS485_TX` (`tx`) | **P3[1]** | Digital Output, Strong | DI del MAX485 #2 |
| `Pin_RS485_DE` (`tx_en`) | **P3[2]** | Digital Output, Strong | DE + RE (unidos) del MAX485 #2 |
| `Pin_DBG_RX` (`rx` de `UART_DBG`) | **P12[6]** | Digital Input, HiZ | KitProg (interno a la placa) |
| `Pin_DBG_TX` (`tx`) | **P12[7]** | Digital Output, Strong | KitProg (interno a la placa) |
| `Pin_LED` | **P2[1]** | Digital Output, sin *HW connection* | LED azul de la placa (parpadea con cada respuesta) |
| `Pin_PWM` (salida `pwm` de `PWM_Fan`) | **P2[3]** | Digital Output, Strong | 220 Ω → Gate del IRLZ44N |
| `Pin_TRIG` | **P2[4]** | Digital Output, Strong, sin *HW connection* | TRIG del HC-SR04 |
| `Pin_ECHO` | **P2[5]** | Digital Input, HiZ, sin *HW connection*, interrupción en ambos flancos → `isr_echo` | ECHO del HC-SR04 (**directo**) |

P2[2] es el botón de la placa (sin uso).

> **Pull-up en RX obligatorio.** Mientras la PSoC transmite, `tx_en` = 1 deshabilita el receptor del MAX485 y RO queda en alta impedancia. Sin pull-up la UART lee basura después de cada respuesta. En PSoC 5LP el pull-up solo actúa si el *Initial drive state* es **High**.

### 1.2 MAX485 #2 (lado esclavo)

| Pin del MAX485 | Va a |
|---|---|
| VCC | VDD (5 V) de la 059 |
| GND | GND de la 059 |
| RO | P3[0] |
| DI | P3[1] |
| DE + RE (unidos) | P3[2] |
| A / B | Bus RS485 (sección 3) |

### 1.3 Sensor HC-SR04

| Pin del HC-SR04 | Va a | Nota |
|---|---|---|
| VCC | 5 V de la 059 | |
| GND | GND común | |
| TRIG | P2[4] | Pulso de 10 µs cada 40 ms |
| ECHO | P2[5] | **Directo, sin divisor 1k/2k**: el puerto está a 5 V. Con el divisor (3.3 V) el umbral CMOS (0.7 × 5 = 3.5 V) puede no detectar el alto |

### 1.4 Ventilador y MOSFET

```
P2[3] ── 220 Ω ── Gate IRLZ44N
                  Source ── GND
                  Drain  ── (−) del ventilador
+24 V ── (+) del ventilador
Diodo de rueda libre en antiparalelo con el ventilador (cátodo a +24 V)
```

- PWM a **25 kHz** (24 MHz / 960), resolución 0.1 %.
- Gate a 5 V (en la ESP32 era 3.3 V): el IRLZ44N conduce mejor, así que el duty de flote puede bajar un poco → recalibrar `ff`.

### 1.5 Tierras

**GND de la 059 = GND de la fuente de 24 V = GND del HC-SR04 = GND del MAX485 #2.** Cómo unirlas sin meter ruido: sección 7.

---

## 2. ESP32 – maestro Modbus v1.1 (`Codigo/Firmware esp32/levitador_esp32_maestro-v1.1`, firmware final)

### 2.1 MAX485 #1 (lado maestro)

| Pin del MAX485 | Pin de la ESP32 | Nota |
|---|---|---|
| DI (Driver In) | **GPIO17** (TX2, UART2) | Datos que transmite la ESP32 |
| RO (Receiver Out) | **GPIO16** (RX2, UART2) | Datos que recibe la ESP32 |
| DE + RE (unidos) | **GPIO4** | HIGH = transmite, LOW = recibe (callbacks `preTransmission` / `postTransmission`) |
| VCC | 5 V o 3.3 V de la ESP32 | Según el módulo (la mayoría, 5 V) |
| GND | GND de la ESP32 | Tierra común |
| A / B | Bus RS485 (sección 3) | |

### 2.2 Puerto de depuración (UART1)

| Pin de la ESP32 | Va a | Nota |
|---|---|---|
| **GPIO25** (TX1) | RX del adaptador USB-TTL | 115200 8N1, segundo monitor serie |
| **GPIO26** (RX1) | — | Reservado, sin uso |
| GND | GND del adaptador USB-TTL | Tierra común |

### 2.3 PC

USB de la ESP32 → PC con LabVIEW (UART0, 115200). Ese puerto **solo** lleva el protocolo; los mensajes legibles van por UART1.

---

## 3. Bus RS485 (entre MAX485 #1 y MAX485 #2)

| Terminal | Conexión | Nota |
|---|---|---|
| A ↔ A | A del #1 con A del #2 | Un hilo de un par trenzado (UTP Cat5e) |
| B ↔ B | B del #1 con B del #2 | El otro hilo del **mismo** par |
| GND ↔ GND | GND de un extremo con GND del otro | Hilo aparte, **obligatorio** |
| Terminación | 120 Ω entre A y B en **cada extremo** | Muchos módulos ya la traen |

> **El GND común no es opcional.** RS485 es diferencial pero no aislado: sin referencia compartida los transceptores pueden salirse de rango y fallar o dañarse.

**Prueba sin la PSoC:** un adaptador USB-RS485 reemplaza al MAX485 #2 + PSoC (A con A, B con B, GND con GND) y en el PC corre un esclavo simulado (simulador propio en Python o Modbus Slave): id 1, RTU, 19200 8N1, IR 0–6, HR 0–5, coils 0–2.

**Síntomas de cableado** (vistos con `etapa 0` en la PSoC): `CRC MALO`, bytes extraños o nada → A/B invertidos, falta el GND común, baudios distintos o falta el pull-up en P3[0].

---

## 4. ESP32 – PID autónomo (`Codigo/PID/levitador_esp32_PID`, usado para la sintonía)

Firmware sin Modbus: la ESP32 mide y controla directamente el levitador.

| Pin de la ESP32 | Va a | Nota |
|---|---|---|
| **GPIO25** | 220 Ω → Gate del IRLZ44N | PWM 25 kHz, 10 bits |
| **GPIO5** | TRIG del HC-SR04 | |
| **GPIO18** | ECHO del HC-SR04 **por divisor 1k/2k** | Baja los 5 V del ECHO a ~3.3 V |
| GND | GND común (fuente 24 V, HC-SR04) | |

Ventilador, MOSFET y diodo: igual que en la sección 1.4, cambiando P2[3] por GPIO25.

> **Ojo con GPIO25:** en este firmware es la salida PWM; en el maestro es el TX de depuración. Si se reflashea la misma ESP32 de uno a otro, cambiar lo que está conectado en GPIO25 (gate del MOSFET ↔ adaptador USB-TTL).

---

## 5. PSoC 5LP – firmware de prueba Modbus (`Codigo/PSoC/Levitador_ModbusTest`)

Mismo cableado de la PSoC que en las secciones 1.1 (solo RS485, depuración y LED) y 1.2, **sin** HC-SR04, MOSFET ni ventilador: la planta es simulada. Si no se pone `Pin_LED`, cambiar `USAR_LED` a `0u` en `main.c`.

---

## 6. Planta física (referencia)

| Elemento | Valor |
|---|---|
| Tubo | 42 cm de alto |
| Pelota | 5.5 cm (icopor + cartones) |
| Ventilador | YXD9225HB24R, 24 V, a 7 cm del piso |
| Sensor | HC-SR04 arriba del tubo, apuntando hacia abajo |
| Fondo calibrado | ≈ 37 cm de distancia sensor → pelota en reposo; setpoint máximo ≈ fondo − 5 cm |

---

## 7. Tierras (GND)

### 7.1 Regla

**Todo lo que intercambia una señal comparte GND.** Un voltaje se mide contra una referencia: sin tierra común, el "alto" de una placa no significa nada para la otra. En este montaje eso incluye a todos los componentes.

| Unión | Por qué es necesaria |
|---|---|
| Fuente de 24 V ↔ PSoC (o ESP32 PID) | El MOSFET conmuta según V<sub>GS</sub>: el Gate lo pone el micro y el Source va al negativo de los 24 V. Sin unión, V<sub>GS</sub> queda indefinido y el ventilador no responde o se comporta raro |
| HC-SR04 ↔ micro | TRIG y ECHO son señales digitales referidas a GND |
| MAX485 #1 ↔ MAX485 #2 | El bus es diferencial, pero los transceptores solo toleran una diferencia de tierra de unos −7 a +12 V entre extremos. Sin el hilo de GND pueden fallar o dañarse |
| ESP32 ↔ adaptador USB-TTL | Como cualquier UART |

**Excepción:** los enlaces aislados (optoacopladores, transceptores RS485 aislados) tienen tierras separadas a propósito. Este montaje no tiene ninguno.

### 7.2 Cómo unirlas: punto de tierra común

El ventilador consume bastante corriente y se conmuta a 25 kHz. Si esa corriente regresa por los cables de tierra de la lógica, mete ruido en el sensor y en el RS485.

```
 −24 V ══════ Source IRLZ44N          (cable grueso y corto: aquí circula la corriente del ventilador)
        ║
        ╚──── un solo hilo ──── GND de la PSoC (punto de tierra común)
                                   ├── GND del HC-SR04
                                   ├── GND del MAX485 #2
                                   └── GND del bus RS485 → MAX485 #1 → GND de la ESP32
```

1. Unir el negativo de la fuente de 24 V y el Source del MOSFET con cable grueso y corto.
2. Desde ese punto, **un solo hilo** al GND de la PSoC.
3. El GND del HC-SR04 y el del MAX485 #2 van al GND de la PSoC, **no** al cable del ventilador.
4. El GND del bus RS485 es solo referencia: por él no debe circular corriente del ventilador.

En la ESP32 PID es igual, cambiando "PSoC" por "ESP32".

### 7.3 USB al mismo PC

Si la ESP32 y la PSoC van por USB al mismo computador, sus tierras ya quedan unidas también a través del PC, además del hilo de GND del bus. En banco, con cables cortos, no causa problemas.

---

## 8. Checklist antes de energizar

- [ ] GND común entre ESP32, MAX485 #1, MAX485 #2, PSoC, HC-SR04 y fuente de 24 V, unido en un punto común (sección 7.2); la corriente del ventilador no pasa por los GND de la lógica.
- [ ] A con A y B con B en el mismo par trenzado; GND por hilo aparte; 120 Ω en ambos extremos.
- [ ] P3[0] de la PSoC con pull-up resistivo e *Initial drive state* = High.
- [ ] VDDIO de la PSoC en 5 V; ECHO directo a P2[5] (sin divisor). En la ESP32 PID, ECHO por divisor.
- [ ] Diodo de rueda libre sobre el ventilador.
- [ ] GPIO25 de la ESP32 conectado a lo que corresponde según el firmware cargado.
