# Levitador Neumático — Firmware ESP32 (maestro Modbus RTU)

Firmware de la ESP32 del proyecto **Levitador Neumático con control PID en PSoC 5LP**. La ESP32 es el puente entre el HMI (LabVIEW) y el controlador (PSoC):

- Recibe **comandos de texto** de LabVIEW por USB y los traduce a escrituras **Modbus RTU** hacia la PSoC.
- Lee la **telemetría** de la PSoC cada 500 ms y la reenvía a LabVIEW en una línea de texto, junto con el **estado del enlace Modbus**.
- Ofrece un **puerto de depuración** independiente con los valores ya des-escalados.

```
 LabVIEW (HMI)  <--USB, 115200 8N1, texto-->  ESP32  <--RS485, 19200 8N1, Modbus RTU-->  PSoC 5LP (PID)
                                                |
                                       UART1 (GPIO25/26)
                                        depuración (USB-TTL)
```

Versión actual (final): **v1.1**, en `levitador_esp32_maestro-v1.1/`. Contrato de comunicación: **Guía ESP32 + HMI v4** (`Recursos/03_Guia_Integracion/`; el contrato no cambió desde la v3).

## Estado

| Etapa | Estado |
|---|---|
| Comunicación bidireccional con LabVIEW (comandos y telemetría) | Funciona (Fases 1 y 2 de la guía, cerradas) |
| Estado del enlace en la trama `DATA` y timeout de 100 ms (v1.1) | Implementado y en uso con la PSoC |
| Integración con la PSoC real (`Codigo/PSoC/psoc_levitador_pid`) | Hecha: MAX485 #2 en la PSoC, telemetría real y setpoint desde el HMI (Fase 3) |

La PSoC final responde en ≈ 3 ms y su timeout de seguridad es de 1.5 s (mayor que el periodo de sondeo de 500 ms, como exige el contrato).

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
| UART2 (`Serial2`) | Bus RS485 / Modbus RTU hacia la PSoC | 19200, 8N1, esclavo con dirección **1**, timeout **100 ms** |

## Mapa de registros Modbus

Es el de la Sección 4 de la guía (sin cambios desde la v2). Las alturas, el setpoint y el error están en **cm × 10** (226 = 22.6 cm).

### Holding registers (escritura FC06)

| Dirección | Nombre | Escala / rango |
|---|---|---|
| 0x0000 | Setpoint | cm × 10, 0–1000 |
| 0x0001 | Kp | × 100 |
| 0x0002 | Ki | × 100 |
| 0x0003 | Kd | × 100 |
| 0x0004 | Modo | 0 = manual, 1 = automático |
| 0x0005 | PWM manual | 0–1000 (= 0.0–100.0 %) |

### Input registers (lectura FC04)

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

**La PSoC debe mantener en IR 0 una copia de solo lectura del holding 0x0000**, de modo que lo que ve LabVIEW sea el setpoint que realmente tiene la planta.

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
| `SP:150` | 0–1000 (15.0 cm) | Holding 0x0000 | Fuera de rango: se recorta. Máximo útil en la PSoC ≈ 310 (fondo − 5 cm); ese límite lo pone el HMI |
| `KP:100` | 0–65535 (1.00) | Holding 0x0001 | Se recorta |
| `KI:60` | 0–65535 (0.60) | Holding 0x0002 | Se recorta |
| `KD:5` | 0–65535 (0.05) | Holding 0x0003 | Se recorta |
| `MODE:1` | 0 o 1 | Holding 0x0004 | Otro valor se interpreta como 1 |
| `PWM:500` | 0–1000 (50.0 %) | Holding 0x0005 | Se recorta |
| `EN:1` | 0 o 1 | Coil 0x0000 | Otro valor se interpreta como 1 |
| `EMG:1` | 0 o 1 | Coil 0x0002 | Otro valor se interpreta como 1 |
| `RST:1` | cualquiera | Coil 0x0001 | Pulso: ON, 50 ms, OFF |

Los valores de ganancias del ejemplo son los de arranque de la PSoC (ajustados en placa); el HMI debe usarlos por defecto. Las claves no distinguen mayúsculas de minúsculas. Aunque LabVIEW ya limita los rangos, la ESP32 los vuelve a validar como segunda barrera; cada ajuste deja un aviso en el puerto de depuración.

**Reglas del parser:**
- Terminador: `\n` o `\r`. Con `\r\n` la segunda mitad se ignora.
- Las líneas vacías se ignoran sin error (las tandas de LabVIEW empiezan con un `\n` suelto).
- Una línea **sin terminador** se procesa igualmente tras **30 ms** sin bytes nuevos. Un comando partido en dos tramos cercanos se arma completo.
- Si el salto de línea llega escrito como texto (barra + `n`), se ignora desde la barra y se avisa.
- Las líneas de más de 64 caracteres se descartan completas.
- En todos los casos anómalos se deja un aviso en el puerto de depuración; conviene corregirlos en el VI, aunque el firmware los tolere.

### Telemetría (ESP32 → LabVIEW)

Cada **500 ms** (2 Hz), **siempre** (falle o no la lectura Modbus), en una sola escritura al puerto y terminada **solo con `\n`**:

```
DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc,enlace
```

| Campo | Tipo / escala | En el HMI |
|---|---|---|
| `sp` | entero, cm × 10 | ÷ 10 → cm |
| `altura` | entero, cm × 10 | ÷ 10 → cm |
| `pwm` | entero, 0–1000 | ÷ 10 → % |
| `error` | entero, cm × 10, con signo | ÷ 10 → cm |
| `estado` | entero, bitmap | — |
| `tramas_ok` | entero, contador | — |
| `errores_crc` | entero, contador | — |
| `enlace` | **texto** ASCII, sin tildes ni comas | Se muestra tal cual |

Ejemplos:

```
DATA,300,226,149,74,3,1520,2,Conexion OK
DATA,300,226,149,74,3,1520,2,Sin conexion (codigo 0xE2: el esclavo no responde)
```

El campo `enlace` es todo lo que va después de la 8.ª coma hasta el salto de línea. En LabVIEW debe separarse como texto: si toda la línea se pasa por *Spreadsheet String to Array* con tipo numérico, ese campo se pierde o da error.

#### Estado del enlace

El campo `enlace` refleja **solo** el sondeo periódico (la lectura FC04). Se calcula con el número de fallos consecutivos y el último código de error de ModbusMaster:

| Situación | Texto |
|---|---|
| Lectura correcta | `Conexion OK` |
| 1–2 fallos, aún sin ninguna lectura válida desde el arranque | `Verificando conexion (codigo 0x..: ...)` |
| 1–2 fallos seguidos | `Conexion inestable (codigo 0x..: ...)` |
| 3+ fallos, el último por timeout (0xE2) | `Sin conexion (codigo 0xE2: el esclavo no responde)` |
| 3+ fallos, el último una excepción Modbus (0x01–0x04) | `Esclavo rechaza peticion (codigo 0x..: ...)` |
| 3+ fallos, el último de otro tipo (0xE0, 0xE1, 0xE3) | `Falla de enlace (codigo 0x..: ...)` |

| Código | Descripción en el texto | Origen |
|---|---|---|
| 0x01 | funcion no soportada por el esclavo | Excepción del esclavo |
| 0x02 | direccion de registro invalida | Excepción del esclavo |
| 0x03 | valor invalido | Excepción del esclavo |
| 0x04 | falla interna del esclavo | Excepción del esclavo |
| 0xE0 | respondio otro esclavo | Detectado por el maestro |
| 0xE1 | funcion inesperada en la respuesta | Detectado por el maestro |
| 0xE2 | el esclavo no responde | Detectado por el maestro (timeout) |
| 0xE3 | CRC invalido | Detectado por el maestro |

- Mientras la lectura falla, los campos numéricos **repiten los últimos valores válidos** (0 si no ha habido ninguna lectura válida desde el arranque). El campo `enlace` indica que están congelados.
- Una sola lectura correcta devuelve el estado a `Conexion OK`.
- `Esclavo rechaza peticion` significa que el cable funciona pero la PSoC no acepta la petición: revisar que implemente el mapa de registros.
- Los errores de **escritura** (comandos `SP`, `KP`, `EMG`...) **no** se reportan en este campo; quedan solo en el puerto de depuración.

## Comportamiento

- **Ciclo de telemetría:** una única transacción FC04 de 7 registros cada 500 ms, seguida siempre de una línea `DATA,`.
- **Timeout del maestro:** **100 ms** (la librería trae 2000 ms; ver [Compilación](#compilación)). Si el esclavo no responde, el `loop()` se bloquea como máximo 100 ms por transacción. Con 3 fallos para declarar la caída, `Sin conexion` aparece unos 1–1.5 s después de perder el enlace.
- **Escrituras:** cada comando es una transacción. Si el esclavo no responde, el comando tarda hasta 100 ms en fallar (`RST`, que son dos escrituras más el pulso, hasta ~250 ms). No hay reintentos.
- **Silencio de LabVIEW:** se mantiene el último setpoint. La ESP32 solo escribe en la PSoC cuando llega un comando; la telemetría sigue leyéndose y enviándose. La PSoC conserva además su propio timeout de seguridad por Modbus (**1.5 s**, mayor que el periodo de sondeo de 500 ms): si salta, pone EN = 0 y el HMI debe volver a enviar `EN:1`.
- **`RST`:** el pulso usa un `delay(50)`, de modo que el `loop()` se bloquea 50 ms. Es aceptable por ser un comando poco frecuente.

## Puerto de depuración (UART1)

Muestra los valores ya des-escalados. Formatos principales:

```
[TEL] SP=30.0cm Alt=22.6cm PWM=14.9% Err=7.4cm Est=0b000011 OK=1520 CRC=2 | Modbus=15ms Periodo=500ms
[CMD] Recibido: SP = 300
  [TX] Holding 0x0 (Setpoint) = 30.0 cm [crudo=300] -> OK
[VALIDACION] SP fuera de rango (1500) -> ajustado a 1000
[CMD] Linea sin terminador (falta '\n'); se procesa por inactividad
[RX] ERROR leyendo input registers 0..6, codigo 0xE2 (el esclavo no responde), fallos seguidos: 1
[ENLACE] Conexion inestable (codigo 0xE2: el esclavo no responde)
[ENLACE] Sin conexion (codigo 0xE2: el esclavo no responde)
[ENLACE] Conexion OK
```

- `[TEL]` sale solo con lecturas correctas, como máximo cada 500 ms. `Modbus` es lo que tardó la lectura del último ciclo y `Periodo` el tiempo real entre ciclos.
- `[RX] ERROR` sale en cada lectura fallida, con la descripción del código y el número de fallos seguidos.
- `[ENLACE]` sale **solo cuando cambia** el texto del campo `enlace`.

## Compilación

1. Arduino IDE con el soporte para placas ESP32 (core de Espressif).
2. Abrir `levitador_esp32_maestro-v1.1/levitador_esp32_maestro-v1.1.ino` (en Arduino la carpeta debe llamarse igual que el sketch; no renombrar una sin la otra).
3. **Copia local de ModbusMaster** (Doc Walker, v2.0.1): **ya viene incluida** en `src/ModbusMaster/` con el timeout reducido. Solo hay que rehacerla si se actualiza la librería:

   ```
   levitador_esp32_maestro-v1.1/
   ├── levitador_esp32_maestro-v1.1.ino
   ├── README.md
   └── src/
       └── ModbusMaster/
           ├── ModbusMaster.h      <- timeout modificado
           ├── ModbusMaster.cpp
           └── util/
               ├── crc16.h
               └── word.h
   ```

   - Copiar el **contenido** de `Documentos/Arduino/libraries/ModbusMaster/src/` en `src/ModbusMaster/`, conservando la carpeta `util`.
   - En `src/ModbusMaster/ModbusMaster.h`, cambiar `ku16MBResponseTimeout = 2000` por `ku16MBResponseTimeout = 100`.
   - El `.ino` la incluye con `#include "src/ModbusMaster/ModbusMaster.h"`. No cambiarlo a `<ModbusMaster.h>`: tomaría la librería instalada, con 2000 ms.
   - Tener ModbusMaster instalada en el Gestor de librerías no estorba.
4. Cerrar y volver a abrir el sketch (el IDE detecta la carpeta `src` al abrir), seleccionar la placa ESP32 y cargar.
5. Comprobación: con la salida detallada activada, ModbusMaster **no** debe aparecer en la lista de librerías usadas desde `Documentos/Arduino/libraries`.

## Pruebas sin la PSoC

Se usa un esclavo Modbus RTU simulado (el simulador propio en Python/Tkinter, o *Modbus Slave* de modbustools.com) en un PC con adaptador USB-RS485:

- Dirección de esclavo 1, Modbus RTU, 19200, 8N1.
- Input registers 0–6, holding registers 0–5 y coils 0–2.
- El simulador debe **espejar el holding 0 en el input register 0**, como lo hará la PSoC; si no, el `sp` de la trama no cambiará al enviar `SP:`.

Para probar el lado de LabVIEW, enviar comandos por USB y verificar en el simulador que el registro o coil correspondiente cambia de inmediato.

Con la PSoC pero sin ventilador: en la consola de la PSoC, `fuente sim` activa un levitador simulado y la cadena HMI ↔ ESP32 ↔ PSoC funciona igual.

Para probar el estado del enlace y el timeout:

- **Desconectar el esclavo** (o cerrar el simulador): las líneas `[RX] ERROR ... 0xE2` deben salir cada 500 ms, y LabVIEW debe pasar de `Conexion inestable` a `Sin conexion` en 1–1.5 s, con los valores numéricos congelados.
- **Con el esclavo desconectado, enviar `EMG:1`:** el `[TX] ... -> ERROR` debe salir casi de inmediato (≤ 100 ms).
- **Reconectar:** el estado vuelve a `Conexion OK` en el siguiente ciclo.
- **Quitar un input register en el simulador** (por ejemplo, dejar solo 0–5): debe aparecer `Esclavo rechaza peticion (codigo 0x02: ...)`.

## Pendiente

Según el plan de trabajo de la guía v4 (Sección 6):

- [x] Implementar en la PSoC el mapa de la Sección 4 (IR 0–6, con IR 0 como espejo del holding 0x0000).
- [x] Confirmar que el timeout de seguridad de la PSoC es mayor que 500 ms (es de 1.5 s).
- [x] Pruebas con la PSoC real (MAX485 #2): telemetría real y setpoint desde el HMI.
- [ ] Fase 2b: desconectar A/B → `Sin conexion` en 1–1.5 s; reconectar → `Conexion OK`; `EMG:1` con el bus desconectado falla en ≤ 100 ms.
- [ ] LabVIEW: separar el 9.º campo (`enlace`) como texto y mostrarlo; aceptar el campo `error` con signo.
- [ ] Prueba de resistencia: 30 minutos continuos sin que crezcan los errores de CRC.
- [ ] Opcional: confirmación de cada comando hacia LabVIEW (hoy los errores de escritura solo van a depuración).
- [ ] Opcional: reemplazar el `delay(50)` de `RST` por una máquina de estados no bloqueante.

## Historial

| Versión | Cambios |
|---|---|
| v1.1 | Campo `enlace` al final de la trama `DATA`; la trama sale siempre y repite los últimos valores válidos si la lectura falla; timeout del maestro de 2000 a 100 ms con copia local de ModbusMaster; periodo de telemetría de 500 ms; líneas `[ENLACE]` y `[RX]` ampliadas en depuración. |
| v1.0 | Primera versión: traductor de comandos, sondeo de 7 input registers y trama `DATA` de 8 campos. |

## Archivos

```
Firmware esp32/
├── README.md                         <- este documento
├── levitador_esp32_maestro-v1.1/     <- FIRMWARE FINAL (v1.1)
│   ├── levitador_esp32_maestro-v1.1.ino
│   └── src/ModbusMaster/             <- copia local de ModbusMaster v2.0.1 con timeout de 100 ms
├── levitador_esp32_maestro/          <- v1.0 (histórico: sondeo cada 100 ms, timeout de 2000 ms, sin campo enlace)
└── Pruebas/                          <- sketches de desarrollo, no se usan en la implementación final
    ├── firmware_esp32_modbus_test-V1 … V9   (evolución del maestro; la V4 no se conservó)
    ├── prueba_comunicacion_esp-labview      (tramas DATA simuladas para probar el VI, formato antiguo "DATA:")
    └── prueba_pwm_version1                  (primera prueba del PWM de 25 kHz al ventilador)
```

La v1.0 parte de la V9 de las pruebas; la v1.1 es la que está cargada en la ESP32 del montaje final.
