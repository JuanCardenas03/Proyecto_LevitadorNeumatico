# PSoC 5LP – Levitador neumático: esclavo Modbus + PID de altura

CY8CKIT-059 · PSoC Creator 4.4 · Mapa de registros de la **Guía ESP32 + HMI v3** (sin cambios de contrato)

Este firmware une el esclavo Modbus de `psoc_modbus_test` con el PID que mejor funcionó en la ESP32 (`levitador_esp32_PID.ino`). El PID es **el mismo, línea por línea**: en el PC se comparó contra el `.ino` durante 3000 pasos y el duty coincide exactamente (ver §7). La única diferencia deliberada está en el paso de manual a automático (§8).

**Valores de arranque (ajustados en placa):** Kp = 1.00 · Ki = 0.60 · Kd = 0.05 · uFF = 49 % · piso = 20 %. Se cambian en `control.h` (`CTRL_*_DEFECTO`).

## 1. Archivos

| Archivo | Qué hace |
|---|---|
| `main.c` | Bucle principal sin bloqueos: Modbus, consola, disparo del sensor cada 40 ms, paso del PID, timeouts de seguridad |
| `control.c/.h` | **PID + feedforward** portado del `.ino`, filtro del HC-SR04, calibración, modos. Sin hardware |
| `hw_levitador.c/.h` | HC-SR04 por interrupción (sin `pulseIn`) y PWM del ventilador a 25 kHz |
| `modbus_slave.c/.h` | Esclavo Modbus (igual al de prueba; solo cambian las ganancias de arranque) |
| `planta_modelo.c/.h` | Levitador simulado, para probar la cadena HMI ↔ ESP32 ↔ PSoC sin ventilador |
| `consola.c/.h` | Comandos por el KitProg (los mismos del monitor de la ESP32-PID) |
| `debug_log.c/.h` | Log legible (sin cambios) |

`plant_sim.c/.h` del proyecto de prueba **se eliminan** (los reemplaza `control.c` + `planta_modelo.c`).

## 2. TopDesign: qué agregar al proyecto de prueba

Se conserva todo lo de `psoc_modbus_test` (`UART_RS485`, `UART_DBG`, `Pin_LED`, pines y `.cydwr`). Se agregan:

| Componente (Catalog) | Nombre | Configuración |
|---|---|---|
| System → Clock | `Clock_PWM` | **24 MHz** (fuente: derivada del reloj del sistema) |
| Digital → Functions → PWM [v3.30] | `PWM_Fan` | Implementation: Fixed Function · Resolution: **16 bit** · PWM mode: One output · **Period: 959** · **Compare mode: Less** · CMP value: 0 · Interrupts: ninguna |
| System → Clock | `Clock_1MHz` | **1 MHz** |
| Digital → Functions → Timer [v2.80] | `Timer_US` | Implementation: Fixed Function · Resolution: **16 bit** · **Period: 65535** · Trigger/Capture: None · Enable mode: Software only · Run mode: Continuous · Interrupts: ninguna |
| Ports and Pins → Digital Output Pin | `Pin_PWM` | Strong drive · conectado a la salida `pwm` de `PWM_Fan` |
| Ports and Pins → Digital Output Pin | `Pin_TRIG` | Strong drive · **desmarcar *HW connection*** |
| Ports and Pins → Digital Input Pin | `Pin_ECHO` | High impedance digital · **desmarcar *HW connection*** · pestaña *Input* → **Interrupt: Both edges** (aparece el terminal `irq`) |
| System → Interrupt | `isr_echo` | Conectado al `irq` de `Pin_ECHO` (Type: DERIVED, por defecto) |

- `Clock_PWM` → entrada `clock` de `PWM_Fan`; `Clock_1MHz` → `clock` de `Timer_US`.
- Si `PWM_Fan` o `Timer_US` muestran terminales `reset` (o `kill`), conéctelos a **Logic Low '0'** (Digital → Logic).
- 24 MHz / 960 = **25 kHz** (lo mismo que la ESP32). El duty se escribe en por mil y queda con resolución de 0.1 %.
- Con el reloj por defecto de la 059 (24 MHz) `Clock_PWM` sale exacto. Si subió el BUS_CLK a 48 MHz, `Clock_PWM` sigue en 24 MHz (divisor 2).

Pines sugeridos (`.cydwr`):

| Pin | Pin físico | Va a |
|---|---|---|
| `Pin_PWM` | **P2[3]** | 220 Ω → Gate del IRLZ44N |
| `Pin_TRIG` | **P2[4]** | TRIG del HC-SR04 |
| `Pin_ECHO` | **P2[5]** | ECHO del HC-SR04 |

(P2[1] es el LED y P2[2] el botón de la placa; RS485 sigue en P3[0..2] y depuración en P12[6..7].)

Luego: reemplace `main.c`, agregue los `.c/.h` nuevos, quite `plant_sim.*`, **Build** y **Program**.

## 3. Cableado del sensor y del ventilador

```
HC-SR04:  VCC → 5 V de la 059   GND → GND   TRIG → P2[4]   ECHO → P2[5]  (directo)
MOSFET:   P2[3] → 220 Ω → Gate IRLZ44N   Source → GND   Drain → (−) del ventilador
          (+) del ventilador → +24 V      diodo de rueda libre sobre el ventilador
GND:      GND de la 059 = GND de la fuente de 24 V = GND del HC-SR04 = GND del MAX485 #2
```

- **ECHO va directo, sin el divisor 1k/2k** de la ESP32: en el `.cydwr` los VDDIO están a 5 V, así que el pin acepta 5 V. Si deja el divisor (3.3 V), el umbral CMOS de un puerto de 5 V (0.7 × 5 = 3.5 V) puede no detectar el alto.
- La compuerta queda a 5 V en lugar de 3.3 V: el IRLZ44N conduce mejor que con la ESP32. Eso puede correr un poco el duty de flote → recalibre `uFF` (paso P2).

## 4. Comportamiento (lo que el equipo de HMI debe saber)

**Mapa de registros: igual a la guía v3.** Lo que cambia es el significado concreto:

| Registro | En este firmware |
|---|---|
| HR0 Setpoint | cm × 10. Si supera el máximo útil (`base − 5 cm`, ≈ 31–32 cm) el PID usa el máximo; IR0 sigue mostrando HR0 |
| HR1–HR3 Kp, Ki, Kd | × 100, en **% de duty por cm** (las unidades del `.ino`). Arranque: **Kp 100, Ki 60, Kd 5** (1.00 / 0.60 / 0.05). Los ejemplos de la guía (40/104/22) eran solo ejemplos: el HMI debería usar estos valores por defecto |
| HR4 Modo | 1 = PID, 0 = manual. Manual → auto **sin salto**: el integrador arranca en `duty manual − uFF`, así la primera salida es el duty manual. `uFF` no cambia. Ganancias, setpoint y piso: los últimos escritos |
| HR5 PWM manual | por mil |
| Coil 0 EN | 0 = ventilador apagado. Lo pone en 0 también la seguridad (ver abajo) |
| Coil 1 RST | flanco de subida → integrador a 0 |
| Coil 2 EMG | 1 = PWM a 0 **en el instante** en que llega la trama; al volver a 0, si EN sigue en 1, el PID retoma sin salto |
| IR1 Altura · IR2 PWM · IR3 Error | IR3 = IR0 − IR1, con signo (complemento a 2) |

**Palabra de estado (IR4):** bit 0 sensor OK (menos de 3 lecturas fallidas seguidas) · bit 1 en objetivo (±5 mm, solo con PID) · bit 2 salida del PID saturada · bit 3 integrador en su límite (±40 %) · bit 4 sin tramas del maestro > 1.5 s · bit 5 pelota más de 2 cm por encima del setpoint máximo.

**Seguridad:**
- **Timeout de comunicación: 1.5 s** sin tramas válidas, habiendo tenido enlace → EN = 0 y ventilador apagado. Al volver el enlace **no arranca solo**: el HMI debe enviar `EN:1`. (Sin ESP32 conectada desde el arranque, la consola funciona normal.)
- **Sensor:** 20 lecturas sin eco seguidas (~0.8 s) con el lazo cerrado → EN = 0 (igual que el `.ino`).
- Al arrancar se calibra el fondo durante ~1.1 s (pelota abajo). Modbus responde mientras tanto.

**Feedforward `uFF`** (49 % por defecto) y **piso** (20 %) no están en el mapa Modbus: **solo cambian por consola** (`ff 49`, `fl 20`) y vuelven a los valores por defecto al reiniciar la PSoC. El piso solo actúa con el PID (en manual y en paro no).

## 5. Consola (KitProg, 115200)

| Comando | Qué hace |
|---|---|
| `sp 15` · `15` · `sp 50%` | Setpoint en cm o en % de la altura útil; pone modo auto y EN = 1 |
| `u 51` | Modo manual con duty en % (buscar el duty de flote) |
| `ff 49` · `fl 20` | Feedforward · piso de duty en vuelo |
| `kp 1.0` · `ki 0.6` · `kd 0.05` | Ganancias (escriben HR1–HR3: el HMI las ve) |
| `p` · `rst` · `en 1` · `emg 1` · `modo 0` | Paro · reset del integrador · coils y modo |
| `a` · `b` · `c` | Rampa del sp en 30 s · barrido PWM 0→100 % · recalibrar fondo |
| `plot 1` · `d` | Líneas `sp:..,altura:..,pwm:..` cada 40 ms para el **Serial Plotter** de Arduino IDE (puerto del KitProg) · agrega `dist` |
| `fuente sim` · `fuente real` | Planta simulada (ventilador apagado) o sensor real |
| `dump` · `log 0/1/2` · `etapa 0/1` · `mute 1` · `reset` | Igual que en el firmware de prueba |

Cada segundo sale un resumen: `[RD] FC04 x2 ok=12 crc=0 | PID SP=15.0cm Alt=15.4cm PWM=54.3% Err=-0.4cm Est=0b000011 I=3.3% uFF=49.0%`

## 6. Procedimiento de prueba en placa

Validar por tramos (regla de oro de la guía). Marque cada paso antes de seguir.

| # | Prueba | Cómo | Qué se espera |
|---|---|---|---|
| P0 | Cadena completa sin ventilador | `fuente sim`, HMI conectado, `SP:150` + `EN:1` | La gráfica del HMI sigue a la planta simulada; `Conexion OK`; campo `Modbus=` de la ESP32 en ~15 ms |
| P1 | Sensor | `fuente real`, ver `[CTRL] Calibracion OK: base = …`; `plot 1` y `d`; subir la pelota con un palito | `altura` sigue a la mano; sin saltos grandes; base ≈ la del `.ino` (~37 cm) |
| P2 | Lazo abierto | `u 40`, `u 45`, `u 50`… hasta que flote a media altura; luego `ff <ese valor>` | Duty de flote cercano a 49 % (valor ajustado en placa) |
| P3 | PID desde consola | `sp 15`, `plot 1` (Serial Plotter); escalones `sp 25`, `sp 8` | Respuesta parecida a la de la ESP32 con las mismas ganancias |
| P4 | PID desde el HMI | SP, ganancias en vivo, `MODE:0` + `PWM:`, `MODE:1`, `RST:1`, `EMG:1` / `EMG:0` | Fase 3, pasos 3–5 de la guía. En `dump`, *pulsos RST* aumenta |
| P5 | Seguridad | Desconectar A/B del RS485 · desconectar ECHO | A los 1.5 s se apaga y el log dice `TIMEOUT DE SEGURIDAD`; a los ~0.8 s `PARO DE SEGURIDAD` |
| P6 | Resistencia | `reset`, 30 min con la pelota en un setpoint | `crc=` no crece; el HMI siempre en `Conexion OK` |

## 7. Pruebas hechas en el PC (carpeta `pruebas/`)

1. **Equivalencia con el `.ino`** (`equivalencia.c`): la medición, `pidPaso()` y `entrarPID()` del `.ino` transcritos tal cual, contra `control.c`, 120 s en lazo cerrado con cambios de setpoint y de ganancias, saturación, ecos atípicos y lecturas perdidas. **Diferencia máxima de duty: 0.0 %** en los 3000 pasos.
2. **Firmware completo simulado** (`sim_host.c`): compila `main.c` y todos los módulos **sin modificar** sobre APIs falsas de PSoC Creator. Emula el HC-SR04 a nivel de pines (TRIG → flancos de ECHO → ISR con el Timer de 1 MHz), el ventilador y la pelota, y una ESP32 que sondea cada 500 ms. **33/33 verificaciones pasan**: valores de arranque, calibración sin bloquear Modbus, IR0 = HR0, error negativo, manual, manual → auto sin salto con `uFF` intacto (dos veces) y `ff` solo por consola, RST, EMG inmediato, timeout de 1.5 s, paro por sensor, excepciones 01/02/03, CRC malo, comandos de consola vistos por el maestro, rampa, plot y planta simulada. Respuesta Modbus: **3 ms** como máximo (contrato: < 100 ms).

La planta simulada **no es el levitador real**: sirve para la lógica, no para juzgar la sintonía. La sintonía se valida en P3.

Para correrlas: `gcc` en Linux/WSL, ver `pruebas/LEEME.md`.

## 8. Diferencias con el `.ino` (fuera del PID, que es idéntico)

- El sensor se mide por interrupción: el paso del PID corre al llegar el eco (máx. 15 ms tras el disparo), con el mismo periodo fijo de 40 ms y el mismo retardo de medición.
- La calibración del fondo no bloquea (se reparte en pasos de 40 ms).
- La rampa va de la altura inicial a `spMax` en 30 s (en el `.ino` se recalculaba desde la altura de cada instante) y escribe el setpoint en HR0 para que el HMI la vea; un SP nuevo la cancela.
- Manual → auto: en el `.ino` el duty manual se adoptaba como `uFF`; aquí inicializa el integrador y `uFF` se conserva (solo lo cambia `ff`). Un `RST:1` vuelve la salida a `uFF`.
- Los paros de seguridad ponen EN = 0 (en vez de esperar un nuevo `sp`).
- El paro por sensor se re-dispara si se habilita con el sensor aún caído (en el `.ino` solo saltaba una vez).
