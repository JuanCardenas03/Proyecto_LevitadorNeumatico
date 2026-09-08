/*
 * ============================================================================
 *  Levitador Neumático - Firmware ESP32 (maestro Modbus RTU)
 *  v3 - Inicio de Fase 2: validación de rangos + puerto de depuración propio
 *  Librería: ModbusMaster (Doc Walker) v2.0.1
 * ============================================================================
 *
 *  Qué cambia respecto a la v2:
 *   - Serial (USB, UART0) queda RESERVADO exclusivamente para el protocolo
 *     de texto con LabVIEW (Sección 3.2/3.3): solo entran comandos
 *     CLAVE:VALOR y solo sale la línea DATA:... Ya no se imprime nada de
 *     depuración ahí, porque ensuciaría lo que LabVIEW intenta parsear.
 *   - Se agrega un puerto de depuración separado sobre UART1 (pines 25/26),
 *     donde SÍ se imprime todo el detalle humano: valores des-escalados
 *     (cm, %, ganancias reales), confirmaciones de escritura, y avisos de
 *     validación de rango.
 *   - Los valores que llegan por comando (SP, KP, KI, KD, MODE, PWM) se
 *     validan/recortan contra los rangos de la Sección 4.3 antes de
 *     escribirlos por Modbus, como defensa adicional aunque LabVIEW ya
 *     limite del otro lado.
 *
 *  Conexiones:
 *   DI  (MAX485)      -> GPIO17 (TX2 / Serial2) - bus RS485 hacia PSoC
 *   RO  (MAX485)      -> GPIO16 (RX2 / Serial2)
 *   DE+RE unidos      -> GPIO4
 *   Depuración (TX)   -> GPIO25 (Serial1) - hacia un adaptador USB-TTL aparte
 *   Depuración (RX)   -> GPIO26 (Serial1) - no se usa por ahora, pero queda listo
 *
 *  Tres UART en uso:
 *   UART0 (Serial)  <- protocolo de texto con LabVIEW (USB integrado)
 *   UART1 (Serial1) <- depuración humana (adaptador USB-TTL externo)
 *   UART2 (Serial2) <- Modbus RTU hacia la PSoC (o Modbus Slave en pruebas)
 * ============================================================================
 */

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
#define MODBUS_SLAVE_ID 1       // Dirección del esclavo (PSoC, o PC en pruebas)
#define MODBUS_BAUD     19200   // Velocidad del bus RS485
// Formato 8N1 (8 bits, sin paridad, 1 de parada) - ver SERIAL_8N1 más abajo

// Parámetros del puerto hacia LabVIEW (Sección 3.1)
#define LABVIEW_BAUD 115200

// Parámetros del puerto de depuración (uso interno, no forma parte del contrato)
#define DEBUG_BAUD 115200

// ---------------------------------------------------------------------------
// Mapa de registros (Sección 4 de la guía - fuente de verdad)
// ---------------------------------------------------------------------------
// Holding registers (escritura, FC06/FC16)
#define HREG_SETPOINT 0x0000  // mm x10
#define HREG_KP       0x0001  // x100
#define HREG_KI       0x0002  // x100
#define HREG_KD       0x0003  // x100
#define HREG_MODE     0x0004  // 0=manual, 1=auto
#define HREG_PWM_MAN  0x0005  // 0-1000

// Input registers (solo lectura, FC04)
#define IREG_ALTURA       0x0000  // mm x10
#define IREG_PWM          0x0001  // 0-1000
#define IREG_ERROR        0x0002  // mm x10, con signo
#define IREG_ESTADO       0x0003  // bitmap
#define IREG_TRAMAS_OK    0x0004  // contador
#define IREG_ERRORES_CRC  0x0005  // contador
#define IREG_COUNT        6       // cantidad total de input registers a leer

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

// Marca de tiempo de la última lectura de telemetría
unsigned long ultimaLectura = 0;
// Cadencia final según la Sección 3.3: 100 ms (10 Hz). Ya no hay motivo
// para mantenerla lenta: la depuración vive en su propio puerto (UART1) y
// no se ve afectada por la velocidad del protocolo con LabVIEW.
const unsigned long PERIODO_TELEMETRIA_MS = 100;

// Buffer para armar la línea de comando que llega por el puerto de LabVIEW
String bufferEntrada = "";

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
  // salvo la línea DATA:... y la lectura de comandos entrantes.
  Serial.begin(LABVIEW_BAUD);

  // UART1: puerto de depuración humana, independiente del protocolo.
  DebugSerial.begin(DEBUG_BAUD, SERIAL_8N1, PIN_DEBUG_RX, PIN_DEBUG_TX);
  DebugSerial.println();
  DebugSerial.println("=== ESP32 Maestro Modbus - Puerto de depuracion ===");
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

  // Nota: aun no implementamos "mantener ultimo setpoint ante timeout de
  // LabVIEW" en este archivo. Eso se añade cuando definamos cómo detectar
  // el silencio del puerto (por ejemplo, un contador de millis() desde el
  // último comando recibido).
}

// ---------------------------------------------------------------------------
// Lectura periódica de telemetría (Sección 4.4 / 3.3)
// ---------------------------------------------------------------------------
void leerTelemetriaPeriodica() {
  if (millis() - ultimaLectura < PERIODO_TELEMETRIA_MS) return;
  ultimaLectura = millis();

  uint8_t resultadoLectura = node.readInputRegisters(IREG_ALTURA, IREG_COUNT);

  if (resultadoLectura == node.ku8MBSuccess) {
    uint16_t altura      = node.getResponseBuffer(IREG_ALTURA);
    uint16_t pwm         = node.getResponseBuffer(IREG_PWM);
    int16_t  error       = (int16_t)node.getResponseBuffer(IREG_ERROR);
    uint16_t estado      = node.getResponseBuffer(IREG_ESTADO);
    uint16_t tramasOk    = node.getResponseBuffer(IREG_TRAMAS_OK);
    uint16_t erroresCrc  = node.getResponseBuffer(IREG_ERRORES_CRC);

    // --- Hacia LabVIEW (Sección 3.3): valores crudos, escalados x10/x1000 ---
    Serial.print("DATA:");
    Serial.print(altura); Serial.print(",");
    Serial.print(pwm); Serial.print(",");
    Serial.print(error); Serial.print(",");
    Serial.print(estado); Serial.print(",");
    Serial.print(tramasOk); Serial.print(",");
    Serial.println(erroresCrc);

    // --- Hacia el puerto de depuración: valores YA des-escalados ---
    DebugSerial.println("--- Telemetria ---");
    DebugSerial.print("  Altura: "); DebugSerial.print(altura / 10.0, 1); DebugSerial.println(" cm");
    DebugSerial.print("  PWM: "); DebugSerial.print(pwm / 10.0, 1); DebugSerial.println(" %");
    DebugSerial.print("  Error: "); DebugSerial.print(error / 10.0, 1); DebugSerial.println(" cm");
    DebugSerial.print("  Estado (bitmap): 0b");
    for (int b = 5; b >= 0; b--) DebugSerial.print((estado >> b) & 0x01);
    DebugSerial.println();
    DebugSerial.print("  Tramas OK: "); DebugSerial.println(tramasOk);
    DebugSerial.print("  Errores CRC: "); DebugSerial.println(erroresCrc);
  } else {
    DebugSerial.print("[RX] ERROR leyendo telemetria, codigo 0x");
    DebugSerial.println(resultadoLectura, HEX);
  }
}

// ---------------------------------------------------------------------------
// Parser de comandos de texto por el puerto de LabVIEW (USB)
// Formato exacto de la Sección 3.2: CLAVE:VALOR terminado en '\n'
// ---------------------------------------------------------------------------
void leerComandoSerie() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      procesarLinea(bufferEntrada);
      bufferEntrada = "";
    } else if (c != '\r') {
      bufferEntrada += c;
    }
  }
}

void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

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
// Validación de rango (Sección 4.3): recorta al límite más cercano y avisa
// por el puerto de depuración si el valor recibido estaba fuera de rango.
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
// Traducciones comando -> Modbus (Sección 4.3 holding registers)
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
// Traducciones comando -> Modbus (Sección 4.5 coils)
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
