/*
 * ============================================================================
 *  Levitador Neumático - Firmware ESP32 (maestro Modbus RTU)
 *  Versión 1.0 - implementación real (parte de la v9 de las pruebas)
 *  Librería: ModbusMaster (Doc Walker) v2.0.1
 * ============================================================================
 *
 *  Función: puente entre LabVIEW (HMI) y la PSoC 5LP (controlador PID).
 *   - Recibe comandos de texto de LabVIEW por USB y los escribe por Modbus RTU.
 *   - Lee la telemetría de la PSoC cada 100 ms y la reenvía a LabVIEW.
 *
 *  Tres UART en uso:
 *   UART0 (Serial)  <-> LabVIEW por USB, 115200 8N1. Solo protocolo de texto.
 *   UART1 (Serial1)  -> Depuración humana (adaptador USB-TTL externo), 115200 8N1.
 *   UART2 (Serial2) <-> Bus RS485 (MAX485) hacia la PSoC, 19200 8N1, esclavo 1.
 *
 *  Conexiones:
 *   DI  (MAX485)  -> GPIO17 (TX2)
 *   RO  (MAX485)  -> GPIO16 (RX2)
 *   DE+RE unidos  -> GPIO4
 *   Depuración    -> TX en GPIO25, RX en GPIO26 (RX sin uso por ahora)
 *
 *  COMANDOS DESDE LABVIEW (Sección 3.2 de la guía):
 *   - Una línea por comando: CLAVE:VALOR\n  (LF = 0x0A).
 *     Claves: SP, KP, KI, KD, MODE, PWM, EN, EMG, RST.
 *     El valor es un entero ya escalado: SP en cm x10 (0-1000), KP/KI/KD x100,
 *     PWM 0-1000, MODE/EN/EMG 0 o 1, RST con cualquier valor (pulso).
 *     Varios comandos por escritura están bien; las líneas vacías se ignoran.
 *   - El parser tolera además: terminador '\r', comandos sin terminador (se
 *     aplican tras 30 ms de silencio) y '\n' escrito como texto. Cada caso
 *     deja un aviso en el puerto de depuración para corregirlo en el VI.
 *   - Los valores se validan/recortan antes de escribirse por Modbus, aunque
 *     LabVIEW ya limite del otro lado.
 *
 *  TELEMETRÍA HACIA LABVIEW:
 *   DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc\n
 *   Valores crudos, tal como están en los registros. El HMI divide entre 10 sp,
 *   altura y error (cm), y el pwm (%). Una sola escritura por trama, solo LF.
 *
 *  CAMBIO RESPECTO A LA GUÍA (acordado con el equipo):
 *   - El setpoint vigente de la planta es el input register IR 0 y los demás
 *     input registers corren una posición. Son 7, en el mismo orden que DATA:
 *       IR0 sp | IR1 altura | IR2 pwm | IR3 error | IR4 estado | IR5 tramas_ok | IR6 errores_crc
 *     La PSoC debe exponer en IR 0 una copia de solo lectura del holding 0x0000.
 *   - Se leen los 7 en UNA sola transacción FC04. El setpoint se ESCRIBE en el
 *     holding 0x0000 con el comando SP.
 *
 *  COMPORTAMIENTO ANTE FALLOS:
 *   - Si falla la lectura Modbus, ese ciclo NO se envía DATA y el error queda
 *     en el puerto de depuración.
 *   - El timeout de respuesta del maestro es el de la librería ModbusMaster
 *     (2000 ms); no se modifica.
 *   - Silencio de LabVIEW: la ESP32 solo escribe en la PSoC cuando llega un
 *     comando, así que se mantiene el último setpoint. La telemetría sigue.
 * ============================================================================
 */

#include <stdio.h>
#include <ModbusMaster.h>

// ---------------------------------------------------------------------------
// Pines (Sección 7 de la guía: acuerdos del equipo)
// ---------------------------------------------------------------------------
#define PIN_RS485_RX   16   // RO del MAX485 -> RX2 de la ESP32
#define PIN_RS485_TX   17   // DI del MAX485 -> TX2 de la ESP32
#define PIN_RS485_DERE 4    // DE+RE unidos del MAX485 -> GPIO4

// Puerto de depuración dedicado (UART1). Pines libres, no chocan con RS485.
#define PIN_DEBUG_RX 26
#define PIN_DEBUG_TX 25

// ---------------------------------------------------------------------------
// Parámetros Modbus (Sección 4.1 de la guía - contrato con la PSoC)
// ---------------------------------------------------------------------------
#define MODBUS_SLAVE_ID 1       // Dirección del esclavo (la PSoC; en pruebas, el simulador)
#define MODBUS_BAUD     19200   // Velocidad del bus RS485
// Formato 8N1 (8 bits, sin paridad, 1 de parada) - ver SERIAL_8N1 más abajo

// Parámetros del puerto hacia LabVIEW (Sección 3.1)
#define LABVIEW_BAUD 115200

// Parámetros del puerto de depuración (uso interno, no forma parte del contrato)
#define DEBUG_BAUD 115200

// ---------------------------------------------------------------------------
// Mapa de registros (Sección 4 de la guía, con la AMPLIACION de IR 0 descrita
// arriba: el setpoint se agrega al inicio de los input registers)
// Escala de alturas/setpoint/error: cm x10 (226 = 22.6 cm), la que usa el HMI.
// ---------------------------------------------------------------------------
// Holding registers (escritura, FC06/FC16; lectura, FC03)
#define HREG_SETPOINT 0x0000  // cm x10
#define HREG_KP       0x0001  // x100
#define HREG_KI       0x0002  // x100
#define HREG_KD       0x0003  // x100
#define HREG_MODE     0x0004  // 0=manual, 1=auto
#define HREG_PWM_MAN  0x0005  // 0-1000

// Input registers (solo lectura, FC04). MAPA AMPLIADO respecto a la guía:
// IR 0 es el setpoint vigente de la planta (copia de solo lectura del holding
// 0x0000) y el resto corre una posición. Mismo orden que la trama DATA.
#define IREG_SP           0x0000  // cm x10 (espejo del holding 0x0000)
#define IREG_ALTURA       0x0001  // cm x10
#define IREG_PWM          0x0002  // 0-1000
#define IREG_ERROR        0x0003  // cm x10, con signo
#define IREG_ESTADO       0x0004  // bitmap
#define IREG_TRAMAS_OK    0x0005  // contador
#define IREG_ERRORES_CRC  0x0006  // contador
#define IREG_COUNT        7       // cantidad total de input registers a leer

// Coils (FC01/FC05)
#define COIL_ENABLE 0x0000
#define COIL_RESET  0x0001
#define COIL_EMG    0x0002

// ---------------------------------------------------------------------------
// Rangos válidos - defensa adicional en la ESP32, aunque LabVIEW ya limite
// estos mismos valores de su lado.
// SP: escala cm x10 (0 a 1000 = 0.0 a 100.0 cm), acordado con el equipo.
// ---------------------------------------------------------------------------
#define SP_MIN        0       // 0.0 cm
#define SP_MAX        1000    // 100.0 cm
#define GANANCIA_MIN  0
#define GANANCIA_MAX  65535   // Kp/Ki/Kd son uint16 completo, sin límite propio
#define PWM_MIN       0
#define PWM_MAX       1000

// ---------------------------------------------------------------------------
// Objetos de los tres puertos
// ---------------------------------------------------------------------------
ModbusMaster node;
HardwareSerial DebugSerial(1); // UART1, para depuración humana

// ---------------------------------------------------------------------------
// Cadencias
// ---------------------------------------------------------------------------
// Telemetría hacia LabVIEW: 100 ms (10 Hz), Sección 3.3 de la guía.
const unsigned long PERIODO_TELEMETRIA_MS = 500;
// Línea de depuración de telemetría (UART1). Más frecuente = más bloqueo.
const unsigned long PERIODO_DEBUG_MS = 500;

// Estado del sondeo
unsigned long ultimaLectura   = 0;     // inicio del último ciclo de telemetría
unsigned long ultimoDebug     = 0;     // última línea de depuración impresa
unsigned long periodoRealMs = 0;       // tiempo real entre los dos últimos ciclos
unsigned long tiempoModbusMs = 0;      // duración de la lectura Modbus del último ciclo

// Parser de comandos (puerto LabVIEW): ver leerComandoSerie()
const unsigned long COMANDO_IDLE_MS  = 30;  // silencio tras el cual una línea sin '\n' se procesa
const unsigned int  COMANDO_MAX_LONG = 64;  // largo máximo de una línea de comando

// Buffer para armar la línea de comando que llega por el puerto de LabVIEW
String bufferEntrada = "";
unsigned long ultimoByteMs = 0;     // instante en que se leyó el último byte de comando
bool descartandoLinea = false;      // true mientras se descarta una línea demasiado larga

// ---------------------------------------------------------------------------
// Callbacks requeridos por ModbusMaster para controlar el DE/RE del MAX485.
// ---------------------------------------------------------------------------
void preTransmission() {
  digitalWrite(PIN_RS485_DERE, HIGH); // modo transmisión
}

void postTransmission() {
  digitalWrite(PIN_RS485_DERE, LOW);  // modo recepción
}

void setup() {
  // UART0 (USB): protocolo de texto con LabVIEW. NO se imprime nada aquí
  // salvo la línea DATA,... y la lectura de comandos entrantes.
  Serial.begin(LABVIEW_BAUD);

  // UART1: puerto de depuración humana, independiente del protocolo.
  DebugSerial.begin(DEBUG_BAUD, SERIAL_8N1, PIN_DEBUG_RX, PIN_DEBUG_TX);
  DebugSerial.println();
  DebugSerial.println("=== Levitador neumatico - ESP32 maestro Modbus v1.0 - Puerto de depuracion ===");
  DebugSerial.println("UART0 (USB) reservado para el protocolo con LabVIEW.");
  DebugSerial.println("Aqui se muestran los valores ya des-escalados.");

  // Pin de control de dirección del MAX485
  pinMode(PIN_RS485_DERE, OUTPUT);
  digitalWrite(PIN_RS485_DERE, LOW); // arranca en modo recepción

  // UART2 para el bus RS485: 19200 baudios, 8N1, pines RX=16, TX=17
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // Inicializar ModbusMaster sobre Serial2, dirección de esclavo = 1
  node.begin(MODBUS_SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  DebugSerial.println("Listo. Esperando comandos por el puerto de LabVIEW (USB)...");
  DebugSerial.println();
}

void loop() {
  leerComandoSerie();
  leerTelemetriaPeriodica();

  // Silencio de LabVIEW: no requiere lógica extra. La ESP32 solo escribe en
  // la PSoC cuando llega un comando, así que sin comandos se mantiene el
  // último setpoint. La telemetría se sigue leyendo y enviando.
}

// ---------------------------------------------------------------------------
// Sondeo periódico (cada PERIODO_TELEMETRIA_MS):
//   UNA sola transaccion FC04 de 7 input registers (IR 0..6): sp, altura, pwm,
//   error, estado, tramas_ok, errores_crc; y envío de la línea
//   DATA,sp,altura,pwm,error,estado,tramas_ok,errores_crc
// ---------------------------------------------------------------------------
void leerTelemetriaPeriodica() {
  unsigned long ahora = millis();
  if (ahora - ultimaLectura < PERIODO_TELEMETRIA_MS) return;
  periodoRealMs = ahora - ultimaLectura;
  ultimaLectura = ahora;

  // --- Lectura Modbus: sp + telemetría en UNA transacción (FC04, IR 0..6) ---
  uint8_t resultadoLectura = node.readInputRegisters(IREG_SP, IREG_COUNT);
  tiempoModbusMs = millis() - ahora;

  if (resultadoLectura != node.ku8MBSuccess) {
    DebugSerial.print("[RX] ERROR leyendo input registers 0..6, codigo 0x");
    DebugSerial.println(resultadoLectura, HEX);
    return;   // sin lectura valida no se envia DATA este ciclo
  }

  uint16_t sp          = node.getResponseBuffer(IREG_SP);
  uint16_t altura      = node.getResponseBuffer(IREG_ALTURA);
  uint16_t pwm         = node.getResponseBuffer(IREG_PWM);
  int16_t  error       = (int16_t)node.getResponseBuffer(IREG_ERROR);
  uint16_t estado      = node.getResponseBuffer(IREG_ESTADO);
  uint16_t tramasOk    = node.getResponseBuffer(IREG_TRAMAS_OK);
  uint16_t erroresCrc  = node.getResponseBuffer(IREG_ERRORES_CRC);

  // --- Hacia LabVIEW: PRIMERO y en una sola escritura ---------------------
  // Valores crudos (sp, altura, error x10; pwm x10 %). Peor caso ~49 bytes.
  // Terminador: SOLO '\n' (no "\r\n"); es lo que espera LabVIEW.
  char linea[80];
  int n = snprintf(linea, sizeof(linea), "DATA,%u,%u,%u,%d,%u,%u,%u\n",
                   (unsigned)sp, (unsigned)altura, (unsigned)pwm, (int)error,
                   (unsigned)estado, (unsigned)tramasOk, (unsigned)erroresCrc);
  if (n > 0 && n < (int)sizeof(linea)) {
    Serial.write((const uint8_t*)linea, (size_t)n);
  }

  // --- Hacia el puerto de depuración: valores YA des-escalados ------------
  // Una sola línea compacta y solo cada PERIODO_DEBUG_MS, para no bloquear el
  // loop (con UART1 a 115200, cada byte impreso cuesta ~87 us de bloqueo).
  if (ahora - ultimoDebug >= PERIODO_DEBUG_MS) {
    ultimoDebug = ahora;
    char bits[7];
    for (int b = 5; b >= 0; b--) bits[5 - b] = ((estado >> b) & 0x01) ? '1' : '0';
    bits[6] = '\0';
    char dbg[200];
    int m = snprintf(dbg, sizeof(dbg),
                     "[TEL] SP=%.1fcm Alt=%.1fcm PWM=%.1f%% Err=%.1fcm Est=0b%s OK=%u CRC=%u | Modbus=%lums Periodo=%lums\r\n",
                     sp / 10.0, altura / 10.0, pwm / 10.0, error / 10.0, bits,
                     (unsigned)tramasOk, (unsigned)erroresCrc, tiempoModbusMs, periodoRealMs);
    if (m > 0) {
      DebugSerial.write((const uint8_t*)dbg, (size_t)(m < (int)sizeof(dbg) ? m : (int)sizeof(dbg) - 1));
    }
  }
}

// ---------------------------------------------------------------------------
// Parser de comandos de texto por el puerto de LabVIEW (USB)
// Formato exacto de la Sección 3.2: CLAVE:VALOR terminado en '\n'
//
// Tolerante a como los mande LabVIEW:
//   - Terminador: '\n' o '\r' (con "\r\n" la segunda mitad llega con el buffer
//     vacío y se ignora). Las líneas vacías se ignoran sin error; las tandas de
//     LabVIEW empiezan con un '\n' suelto (por el "%.;\n" del formato).
//   - Si una línea queda SIN terminador y pasan COMANDO_IDLE_MS sin bytes
//     nuevos, se procesa igual. Así un comando suelto (MODE, EN, EMG...) no
//     espera a que la siguiente tanda "lo empuje" con su '\n' inicial.
//   - Líneas de más de COMANDO_MAX_LONG caracteres se descartan completas
//     (hasta el siguiente terminador o silencio) para no acumular ruido.
// ---------------------------------------------------------------------------
void leerComandoSerie() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    ultimoByteMs = millis();
    if (c == '\n' || c == '\r') {
      if (descartandoLinea) {
        descartandoLinea = false;
      } else if (bufferEntrada.length() > 0) {
        procesarLinea(bufferEntrada);
      }
      bufferEntrada = "";
    } else if (!descartandoLinea) {
      if (bufferEntrada.length() >= COMANDO_MAX_LONG) {
        DebugSerial.println("[CMD] Linea demasiado larga, se descarta");
        bufferEntrada = "";
        descartandoLinea = true;
      } else {
        bufferEntrada += c;
      }
    }
  }

  // Sin terminador y sin bytes nuevos: procesar lo pendiente por inactividad.
  if ((bufferEntrada.length() > 0 || descartandoLinea) &&
      millis() - ultimoByteMs >= COMANDO_IDLE_MS) {
    if (descartandoLinea) {
      descartandoLinea = false;
    } else {
      DebugSerial.println("[CMD] Linea sin terminador (falta '\\n'); se procesa por inactividad");
      procesarLinea(bufferEntrada);
    }
    bufferEntrada = "";
  }
}

void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  // Si LabVIEW escribe el salto de línea como TEXTO (barra + n) en vez del
  // carácter real, la línea trae una '\'. Se ignora desde ahí y se avisa.
  int barra = linea.indexOf('\\');
  if (barra >= 0) {
    DebugSerial.println("[CMD] Aviso: la linea contiene '\\' (posible '\\n' escrito como texto en LabVIEW); se ignora desde ahi");
    linea = linea.substring(0, barra);
    linea.trim();
    if (linea.length() == 0) return;
  }

  int idx = linea.indexOf(':');
  if (idx < 0) {
    DebugSerial.print("[CMD] Formato invalido (falta ':'): ");
    DebugSerial.println(linea);
    return;
  }

  String clave = linea.substring(0, idx);
  String valorStr = linea.substring(idx + 1);
  clave.trim();
  valorStr.trim();
  clave.toUpperCase();

  long valor = valorStr.toInt();

  DebugSerial.print("[CMD] Recibido: ");
  DebugSerial.print(clave);
  DebugSerial.print(" = ");
  DebugSerial.println(valor);

  if (clave == "SP") {
    uint16_t crudo = (uint16_t)clampValor(valor, SP_MIN, SP_MAX, "SP");
    escribirHolding(HREG_SETPOINT, crudo, "Setpoint", String(crudo / 10.0, 1) + " cm");
    // El valor que realmente quedó en la planta vuelve en IR 0 en el siguiente ciclo.
  } else if (clave == "KP") {
    uint16_t crudo = (uint16_t)clampValor(valor, GANANCIA_MIN, GANANCIA_MAX, "KP");
    escribirHolding(HREG_KP, crudo, "Kp", String(crudo / 100.0, 2));
  } else if (clave == "KI") {
    uint16_t crudo = (uint16_t)clampValor(valor, GANANCIA_MIN, GANANCIA_MAX, "KI");
    escribirHolding(HREG_KI, crudo, "Ki", String(crudo / 100.0, 2));
  } else if (clave == "KD") {
    uint16_t crudo = (uint16_t)clampValor(valor, GANANCIA_MIN, GANANCIA_MAX, "KD");
    escribirHolding(HREG_KD, crudo, "Kd", String(crudo / 100.0, 2));
  } else if (clave == "MODE") {
    uint16_t crudo = (uint16_t)normalizarBinario(valor, "MODE");
    escribirHolding(HREG_MODE, crudo, "Modo", crudo == 1 ? "Automatico" : "Manual");
  } else if (clave == "PWM") {
    uint16_t crudo = (uint16_t)clampValor(valor, PWM_MIN, PWM_MAX, "PWM");
    escribirHolding(HREG_PWM_MAN, crudo, "PWM manual", String(crudo / 10.0, 1) + " %");
  } else if (clave == "EN") {
    int estado = normalizarBinario(valor, "EN");
    escribirCoil(COIL_ENABLE, estado != 0, "Enable", estado ? "Habilitado" : "Deshabilitado");
  } else if (clave == "EMG") {
    int estado = normalizarBinario(valor, "EMG");
    escribirCoil(COIL_EMG, estado != 0, "Emergencia", estado ? "EMERGENCIA ACTIVADA" : "Emergencia desactivada");
  } else if (clave == "RST") {
    pulsoResetIntegrador();
  } else {
    DebugSerial.print("[CMD] Clave desconocida: ");
    DebugSerial.println(clave);
  }
}

// ---------------------------------------------------------------------------
// Validación de rango: recorta al límite más cercano y avisa por el puerto
// de depuración si el valor recibido estaba fuera de rango.
// Defensa adicional aunque LabVIEW ya limite estos valores de su lado.
// ---------------------------------------------------------------------------
long clampValor(long valor, long minV, long maxV, const char* nombre) {
  long ajustado = valor;
  if (ajustado < minV) ajustado = minV;
  if (ajustado > maxV) ajustado = maxV;
  if (ajustado != valor) {
    DebugSerial.print("[VALIDACION] ");
    DebugSerial.print(nombre);
    DebugSerial.print(" fuera de rango (");
    DebugSerial.print(valor);
    DebugSerial.print(") -> ajustado a ");
    DebugSerial.println(ajustado);
  }
  return ajustado;
}

// Para campos que deben ser estrictamente 0 o 1 (MODE, EN, EMG).
// Si llega otro valor, se interpreta como "verdadero" (1) y se avisa.
int normalizarBinario(long valor, const char* nombre) {
  if (valor != 0 && valor != 1) {
    DebugSerial.print("[VALIDACION] ");
    DebugSerial.print(nombre);
    DebugSerial.print(": valor ");
    DebugSerial.print(valor);
    DebugSerial.println(" no es 0/1, se interpreta como 1 (verdadero)");
    return 1;
  }
  return (int)valor;
}

// ---------------------------------------------------------------------------
// Traducciones comando -> Modbus (holding registers)
// valorHumano ya viene formateado y des-escalado, para el puerto de depuración.
// ---------------------------------------------------------------------------
void escribirHolding(uint16_t direccion, uint16_t valorCrudo, const char* nombre, const String& valorHumano) {
  uint8_t resultado = node.writeSingleRegister(direccion, valorCrudo);
  DebugSerial.print("  [TX] Holding 0x");
  DebugSerial.print(direccion, HEX);
  DebugSerial.print(" (");
  DebugSerial.print(nombre);
  DebugSerial.print(") = ");
  DebugSerial.print(valorHumano);
  DebugSerial.print(" [crudo=");
  DebugSerial.print(valorCrudo);
  DebugSerial.print("] -> ");
  DebugSerial.println(resultado == node.ku8MBSuccess ? "OK" : "ERROR");
}

// ---------------------------------------------------------------------------
// Traducciones comando -> Modbus (coils)
// ---------------------------------------------------------------------------
void escribirCoil(uint16_t direccion, bool estado, const char* nombre, const String& valorHumano) {
  uint8_t resultado = node.writeSingleCoil(direccion, estado ? 1 : 0);
  DebugSerial.print("  [TX] Coil 0x");
  DebugSerial.print(direccion, HEX);
  DebugSerial.print(" (");
  DebugSerial.print(nombre);
  DebugSerial.print(") = ");
  DebugSerial.print(valorHumano);
  DebugSerial.print(" -> ");
  DebugSerial.println(resultado == node.ku8MBSuccess ? "OK" : "ERROR");
}

// ---------------------------------------------------------------------------
// RST:1 es un PULSO (Sección 4.5): reinicia el integrador del PID.
// Escribimos el coil en ON y lo devolvemos a OFF poco después, para que
// del lado del esclavo se vea un flanco de subida y no quede "trabado".
// ---------------------------------------------------------------------------
void pulsoResetIntegrador() {
  uint8_t r1 = node.writeSingleCoil(COIL_RESET, 1);
  DebugSerial.print("  [TX] Coil 0x");
  DebugSerial.print(COIL_RESET, HEX);
  DebugSerial.print(" (Reset integrador) = ON -> ");
  DebugSerial.println(r1 == node.ku8MBSuccess ? "OK" : "ERROR");

  delay(50); // pulso breve; ver nota abajo

  uint8_t r2 = node.writeSingleCoil(COIL_RESET, 0);
  DebugSerial.print("  [TX] Coil 0x");
  DebugSerial.print(COIL_RESET, HEX);
  DebugSerial.print(" (Reset integrador) = OFF -> ");
  DebugSerial.println(r2 == node.ku8MBSuccess ? "OK" : "ERROR");
  // Nota: este delay(50) bloquea el loop brevemente. Es aceptable porque
  // RST es un comando poco frecuente, pero si mas adelante notan que
  // afecta la cadencia de telemetria, se puede rehacer sin delay usando
  // una maquina de estados no bloqueante.
}
