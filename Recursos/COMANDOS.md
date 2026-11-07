# Levitador neumático – Referencia unificada de comandos

Todos los comandos de texto que aceptan los firmwares del proyecto, separados por firmware.
Fuente: el código de cada consola/parser (`consola.c`, `procesarLinea()`, `leerComandos()`), no los comentarios.

| # | Firmware | Carpeta / archivo | Puerto de comandos | Formato |
|---|---|---|---|---|
| 1 | PSoC – esclavo Modbus + PID | `psoc_levitador_pid/` | KitProg USB-UART, 115200 8N1 | `cmd valor` o `cmd=valor` |
| 2 | ESP32 – maestro Modbus v1.1 | `levitador_esp32_maestro-v1.1/` | USB (UART0), 115200 8N1 | `CLAVE:VALOR` |
| 3 | ESP32 – PID autónomo | `levitador_esp32_PID.ino` | USB, 115200 8N1 | `cmd=valor` o letra |
| 4 | PSoC – prueba Modbus | `psoc_modbus_test/` | KitProg USB-UART, 115200 8N1 | `cmd valor` |

---

## 1. PSoC – esclavo Modbus + PID (`psoc_levitador_pid`)

**Puerto:** KitProg USB-UART (COMx), 115200 8N1. Fin de línea CR o LF. La PSoC hace eco (desactivar *local echo* en PuTTY/Tera Term).
**Sintaxis:** `sp 15` y `sp=15` son equivalentes. No distingue mayúsculas. Backspace funciona. Máx. 47 caracteres por línea.
**Importante:** los comandos de control escriben los **mismos registros** que el maestro Modbus, así que el HMI ve los cambios (excepto `ff` y `fl`, que no están en el mapa).

### 1.1 Control

| Comando | Argumento | Efecto | Registros que toca |
|---|---|---|---|
| `sp <cm>` · `<cm>` | 0–100 cm (decimales OK) | Setpoint en cm. Pone modo auto y EN = 1 | HR0 = cm×10, HR4 = 1, Coil0 = 1 |
| `sp <n>%` · `<n>%` | 0–100 % | Setpoint en % de la altura útil (`spMax`) | Igual que arriba |
| `u <pct>` | 0–100 % | Modo MANUAL con ese duty (buscar el duty de flote). EN = 1 | HR5 = pct×10, HR4 = 0, Coil0 = 1 |
| `ff <pct>` | 0–100 % | Feedforward de sostenimiento `uFF` (defecto 51 %) | — (no está en Modbus) |
| `fl <pct>` | 0–90 % | Piso de duty en vuelo (defecto 0 %) | — (no está en Modbus) |
| `kp <v>` | 0–655.35 | Ganancia proporcional (% de duty por cm) | HR1 = v×100 |
| `ki <v>` | 0–655.35 | Ganancia integral | HR2 = v×100 |
| `kd <v>` | 0–655.35 | Ganancia derivativa | HR3 = v×100 |
| `p` · `paro` | — | Paro | Coil0 = 0 |
| `en 0\|1` | 0 / ≠0 | Habilitar control | Coil0 |
| `emg 0\|1` | 0 / ≠0 | Emergencia (PWM a 0 inmediato; al volver a 0 retoma sin salto si EN = 1) | Coil2 |
| `modo 0\|1` | 0 / ≠0 | 0 = manual, 1 = auto (PID) | HR4 |
| `rst` | — | Reinicia el integrador | — (interno) |
| `a` · `rampa` | — | Rampa del sp desde la altura actual hasta `spMax` en 30 s. Un SP nuevo la cancela | HR4 = 1, Coil0 = 1, HR0 se actualiza en la rampa |
| `b` · `barrido` | — | Barrido PWM 0 → 100 % en 30 s, lazo abierto (caracterización). Al terminar EN = 0 | Coil0 = 1 |
| `c` · `cal` | — | Recalibra el fondo (pelota abajo, ventilador apagado; espera 1.5 s y muestrea) | Coil0 = 0 |
| `plot 0\|1` | 0 / ≠0 | Líneas `sp:..,altura:..,pwm:..` cada 40 ms para el Serial Plotter | — |
| `d` | — | Alterna agregar `dist:..` (distancia cruda) a la salida de `plot` | — |
| `fuente sim` · `fuente real` | texto | Planta simulada (ventilador apagado) o HC-SR04 real. Pone EN = 0 y recalibra | Coil0 = 0 |

### 1.2 Comunicación y diagnóstico

| Comando | Argumento | Efecto |
|---|---|---|
| `etapa 0\|1` | 0 / ≠0 | 0 = sniffer pasivo (no responde), 1 = esclavo Modbus (defecto) |
| `log 0\|1\|2` | 0, 1, 2 | 0 silencio · 1 escrituras + resumen por segundo · 2 todo en hex |
| `mute 0\|1` | 0 / ≠0 | 1 = la PSoC no responde (la ESP32 verá timeout `0xE2`) |
| `dump` | — | Imprime HR, IR, coils, estado del control (modo, sp, altura, duty, I, uFF, piso, base, spMax, fallos) y configuración |
| `reset` | — | Contadores de tramas OK y errores CRC (IR5, IR6) a cero |
| `help` · `?` | — | Ayuda |

**Notas**
- Un `sp` mayor que `spMax` (≈ base − 5 cm) se guarda tal cual en HR0, pero el PID usa `spMax`.
- `fuente` con cualquier texto distinto de `sim` selecciona el sensor real.
- Comandos que este firmware **no** tiene (sí están en el de prueba): `alt`, `err`, `est`, `sim`.

---

## 2. ESP32 – maestro Modbus v1.1 (`levitador_esp32_maestro-v1.1`)

**Puerto de comandos:** UART0 (USB), 115200 8N1. Es el puerto de **LabVIEW**; no se imprime nada humano ahí.
**Puerto de depuración:** UART1, TX en GPIO25 (adaptador USB-TTL), 115200. **No acepta comandos** (RX en GPIO26 sin uso).
**Sintaxis:** una línea por comando `CLAVE:VALOR\n`. La clave no distingue mayúsculas. Valores enteros ya escalados.

### 2.1 Comandos

| Clave | Valor (crudo) | Rango (se recorta) | Escala | Acción Modbus |
|---|---|---|---|---|
| `SP` | entero | 0–1000 | cm × 10 (`SP:150` = 15.0 cm) | FC06 → HR0 |
| `KP` | entero | 0–65535 | × 100 (`KP:100` = 1.00) | FC06 → HR1 |
| `KI` | entero | 0–65535 | × 100 | FC06 → HR2 |
| `KD` | entero | 0–65535 | × 100 | FC06 → HR3 |
| `MODE` | 0 / 1 | otro valor → 1 | 0 = manual, 1 = auto | FC06 → HR4 |
| `PWM` | entero | 0–1000 | por mil (`PWM:510` = 51.0 %) | FC06 → HR5 |
| `EN` | 0 / 1 | otro valor → 1 | 1 = habilitado | FC05 → Coil0 |
| `EMG` | 0 / 1 | otro valor → 1 | 1 = emergencia | FC05 → Coil2 |
| `RST` | cualquiera | — | pulso | FC05 Coil1 = ON, 50 ms, Coil1 = OFF |

**Tolerancias del parser** (cada una deja un aviso en depuración):
- Terminador `\n` o `\r` (con `\r\n` la segunda mitad se ignora). Líneas vacías se ignoran.
- Línea sin terminador: se procesa tras **30 ms** de silencio.
- `\n` escrito como texto (barra + n): se ignora desde la `\`.
- Líneas de más de 64 caracteres se descartan. Clave desconocida o falta `:` → aviso, sin acción.
- Varios comandos en una sola escritura están bien.

### 2.2 Telemetría que envía (cada 500 ms, por UART0)

```
DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc,enlace\n
```

Valores crudos de IR0–IR6 (sp, altura y error en cm×10; pwm por mil; error con signo). Si la lectura falla se repiten los últimos valores válidos. `enlace`:
`Conexion OK` · `Verificando conexion (...)` · `Conexion inestable (...)` · `Sin conexion (...)` · `Esclavo rechaza peticion (...)` · `Falla de enlace (...)`.

---

## 3. ESP32 – PID autónomo (`levitador_esp32_PID.ino`)

**Puerto:** USB, 115200. Monitor serie de Arduino IDE con **"Nueva línea"** (lee hasta `\n`).
**Sintaxis:** comandos con valor **solo con `=`** (`sp=15`; `sp 15` no funciona). Espacios alrededor del `=` se toleran. No distingue mayúsculas.
**Sin Modbus:** todo es local a la ESP32.

### 3.1 Comandos con valor

| Comando | Rango | Efecto |
|---|---|---|
| `sp=<cm>` · `<cm>` | 0–`spMax` (se recorta y avisa) | Setpoint en cm; activa PID (sin salto) |
| `sp=<n>%` · `<n>%` | 0–100 % | Setpoint en % de la altura útil; activa PID |
| `u=<pct>` | 0–100 % | Modo MANUAL con ese duty |
| `ff=<pct>` | 0–100 % | Feedforward de sostenimiento `uFF` (defecto 51 %) |
| `fl=<pct>` | 0–90 % | Piso de duty en vuelo |
| `kp=<v>` · `ki=<v>` · `kd=<v>` | ≥ 0 (negativo → 0) | Ganancias en caliente (defecto 1.0 / 0.6 / 0.2) |

Clave desconocida con `=` → muestra la ayuda.

### 3.2 Comandos de una letra

Solo se evalúa la **primera letra** (`paro` = `p`, `abc` = `a`).

| Letra | Efecto |
|---|---|
| `p` | Paro (duty 0, integrador a 0) |
| `a` | Rampa del sp desde la altura actual hasta `spMax` en 30 s; luego PID sostiene |
| `b` | Barrido PWM 0 → 100 % en 30 s (caracterización); al terminar, paro |
| `c` | Paro + espera 1.5 s (bloqueante) + recalibrar fondo |
| `d` | Alterna agregar `dist:..` a la salida del plotter |
| `?` (o cualquier otra) | Ayuda + estado actual (uFF, Kp, Ki, Kd, fl, SP, spMax) |

**Salida continua** (cada 40 ms, siempre activa): `sp:..,altura:..,pwm:..[,dist:..]` para el Serial Plotter. Mensajes informativos empiezan con `#`.

---

## 4. PSoC – firmware de prueba Modbus (`psoc_modbus_test`)

**Puerto:** KitProg USB-UART, 115200 8N1, CR o LF, con eco.
**Sintaxis:** `cmd valor` separado por espacio (sin `=`). Los valores enteros aceptan decimal, `0x..` y negativos.
**Uso:** validar el tramo ESP32 → RS485 → PSoC con planta simulada (mapa de la guía v2).

| Comando | Argumento | Efecto |
|---|---|---|
| `etapa 0\|1` | 0 / ≠0 | 0 = sniffer pasivo, 1 = esclavo Modbus (defecto) |
| `log 0\|1\|2` | 0–2 | 0 silencio · 1 escrituras + resumen/s · 2 todo en hex |
| `alt <v>` | cm × 10 | Fija la altura (`alt 226` = 22.6 cm) |
| `err <v>` | cm × 10, con signo | Fuerza el error (`err -35`) |
| `est <v>` | 0x00–0x3F | Fuerza la palabra de estado (`est 0x21`, `est 0x11`) |
| `sim` | — | Quita todo lo forzado (vuelve la planta simulada) |
| `mute 0\|1` | 0 / ≠0 | 1 = no responde (simula enlace caído) |
| `dump` | — | Imprime registros, configuración y valores forzados |
| `reset` | — | Contadores de tramas y CRC a cero |
| `help` · `?` | — | Ayuda |

No tiene comandos de control (`sp`, `kp`, …): el control se prueba desde el HMI/ESP32.

---

## 5. Equivalencias entre firmwares

| Acción | PSoC PID (consola) | ESP32 maestro (LabVIEW) | ESP32 PID | PSoC prueba |
|---|---|---|---|---|
| Setpoint 15 cm | `sp 15` · `15` | `SP:150` | `sp=15` · `15` | — |
| Setpoint en % | `sp 50%` | — | `sp=50%` | — |
| Kp = 1.0 | `kp 1.0` | `KP:100` | `kp=1.0` | — |
| Ki = 0.6 | `ki 0.6` | `KI:60` | `ki=0.6` | — |
| Kd = 0.2 | `kd 0.2` | `KD:20` | `kd=0.2` | — |
| Manual a 51 % | `u 51` | `MODE:0` + `PWM:510` + `EN:1` | `u=51` | — |
| Modo auto | `modo 1` | `MODE:1` | (implícito con `sp`) | — |
| Habilitar | `en 1` | `EN:1` | (implícito con `sp`/`u`) | — |
| Paro | `p` · `en 0` | `EN:0` | `p` | — |
| Emergencia | `emg 1` / `emg 0` | `EMG:1` / `EMG:0` | — | — |
| Reset integrador | `rst` | `RST:1` | — | — |
| Feedforward | `ff 51` | — | `ff=51` | — |
| Piso de duty | `fl 0` | — | `fl=0` | — |
| Rampa / barrido / calibrar | `a` · `b` · `c` | — | `a` · `b` · `c` | — |
| Serial Plotter | `plot 1` (+ `d`) | — | siempre activo (+ `d`) | — |
| Planta simulada | `fuente sim` | — | — | `sim` (+ `alt`/`err`/`est`) |
| Sniffer / mute / log | `etapa` · `mute` · `log` | — | — | `etapa` · `mute` · `log` |
| Volcado / contadores | `dump` · `reset` | — | — | `dump` · `reset` |
