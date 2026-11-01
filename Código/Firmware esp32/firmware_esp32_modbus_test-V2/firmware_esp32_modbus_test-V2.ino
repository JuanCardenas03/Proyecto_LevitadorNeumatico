/*
 * ============================================================================
 *  Levitador Neumático - Firmware ESP32 (maestro Modbus RTU)
 *  FASE 1 (v2): ESP32 sola, contra PC (Modbus Slave / adaptador USB-RS485)
 *  Ahora con parser de comandos de texto por el monitor serie (Sección 3.2)
 *  Librería: ModbusMaster (Doc Walker) v2.0.1
 * ============================================================================
 *
 *  Objetivo de este sketch:
 *   - Simular a LabVIEW escribiendo comandos de texto CLAVE:VALOR por el
 *     monitor serie USB (misma sintaxis exacta que usará LabVIEW después:
 *     SP, KP, KI, KD, MODE, PWM, EN, EMG, RST).
 *   - Cada comando se traduce a la escritura Modbus correspondiente
 *     (holding register o coil) según el mapa de la Sección 4 de la guía.
 *   - En paralelo, sigue leyendo periódicamente los 6 input registers de
 *     telemetría y los imprime, para verificar visualmente en Modbus Slave
 *     que el valor realmente cambió del lado del "esclavo".
 *
 *  Conexiones (según acuerdo del equipo):
 *   DI  (MAX485) -> GPIO17 (TX2)
 *   RO  (MAX485) -> GPIO16 (RX2)
 *   DE+RE unidos -> GPIO4
 *
 *  Se usa Serial2 (UART2) para el bus RS485, dejando Serial (USB, UART0)
 *  libre para el protocolo de texto (monitor serie ahora, LabVIEW después).
 * ============================================================================
 */

#include <ModbusMaster.h>

// ---------------------------------------------------------------------------
// Pines (Sección 7 de la guía: acuerdos del equipo)
// ---------------------------------------------------------------------------
#define PIN_RS485_RX   16   // RO del MAX485 -> RX2 de la ESP32
#define PIN_RS485_TX   17   // DI del MAX485 -> TX2 de la ESP32
#define PIN_RS485_DERE 4    // DE+RE unidos del MAX485 -> GPIO4

// ---------------------------------------------------------------------------
// Parámetros Modbus (Sección 4.1 de la guía - contrato con la PSoC)
// ---------------------------------------------------------------------------
#define MODBUS_SLAVE_ID 1       // Dirección del esclavo (PSoC, o PC en pruebas)
#define MODBUS_BAUD     19200   // Velocidad del bus RS485
// Formato 8N1 (8 bits, sin paridad, 1 de parada) - ver SERIAL_8N1 más abajo

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
// Objeto ModbusMaster
// ---------------------------------------------------------------------------
ModbusMaster node;

// Marca de tiempo de la última lectura de telemetría
unsigned long ultimaLectura = 0;
// NOTA: la Sección 3.3 pide 100 ms (10 Hz) para el envío real a LabVIEW.
// Mientras probamos a mano por el monitor serie, usamos 1000 ms para que
// el texto no se mezcle con lo que estás escribiendo. Cambiar a 100 en Fase 2.
const unsigned long PERIODO_TELEMETRIA_MS = 1000;

// Buffer para armar la línea de comando que llega por el monitor serie
String bufferEntrada = "";

// ---------------------------------------------------------------------------
// Callbacks requeridos por ModbusMaster para controlar el DE/RE del MAX485.
// La librería los llama automáticamente antes y después de cada transacción.
// ---------------------------------------------------------------------------
void preTransmission() {
  digitalWrite(PIN_RS485_DERE, HIGH); // modo transmisión
}

void postTransmission() {
  digitalWrite(PIN_RS485_DERE, LOW);  // modo recepción
}

void setup() {
  // USB para depuración (monitor serie)
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println();
  Serial.println("=== ESP32 Maestro Modbus - Fase 1: prueba contra PC ===");

  // Pin de control de dirección del MAX485
  pinMode(PIN_RS485_DERE, OUTPUT);
  digitalWrite(PIN_RS485_DERE, LOW); // arranca en modo recepción

  // UART2 para el bus RS485: 19200 baudios, 8N1, pines RX=16, TX=17
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // Inicializar ModbusMaster sobre Serial2, dirección de esclavo = 1
  node.begin(MODBUS_SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Listo. Escribe comandos como en la Seccion 3.2, por ejemplo:");
  Serial.println("  SP:300   (setpoint = 30.0 cm)");
  Serial.println("  KP:40    (Kp = 0.40)");
  Serial.println("  MODE:1   (modo automatico)");
  Serial.println("  EN:1     (habilitar control)");
  Serial.println("  RST:1    (pulso de reset del integrador)");
  Serial.println("Verifica el cambio en Modbus Slave (ventana de Holding Registers / Coils).");
  Serial.println();
}

void loop() {
  leerComandoSerie();
  leerTelemetriaPeriodica();

  // Nota: aun no implementamos "mantener ultimo setpoint ante timeout de
  // LabVIEW" porque en esta fase el "LabVIEW" eres tu escribiendo a mano;
  // no tiene sentido un timeout cuando no hay un flujo continuo esperado.
  // Esa lógica se añade en la Fase 2, cuando exista un puerto real hacia
  // el HMI cuyo silencio haya que detectar.
}

// ---------------------------------------------------------------------------
// Lectura periódica de telemetría (Sección 4.4 / 3.3)
// ---------------------------------------------------------------------------
void leerTelemetriaPeriodica() {
  if (millis() - ultimaLectura < PERIODO_TELEMETRIA_MS) return;
  ultimaLectura = millis();

  uint8_t resultadoLectura = node.readInputRegisters(IREG_ALTURA, IREG_COUNT);

  Serial.print("[RX] Telemetria -> ");
  if (resultadoLectura == node.ku8MBSuccess) {
    uint16_t altura      = node.getResponseBuffer(IREG_ALTURA);
    uint16_t pwm         = node.getResponseBuffer(IREG_PWM);
    int16_t  error       = (int16_t)node.getResponseBuffer(IREG_ERROR);
    uint16_t estado      = node.getResponseBuffer(IREG_ESTADO);
    uint16_t tramasOk    = node.getResponseBuffer(IREG_TRAMAS_OK);
    uint16_t erroresCrc  = node.getResponseBuffer(IREG_ERRORES_CRC);

    // Misma línea DATA que finalmente irá hacia LabVIEW (Sección 3.3),
    // así probamos también ese formato de salida desde ya.
    Serial.print("DATA,12,");
    Serial.print(altura); Serial.print(",");
    Serial.print(pwm); Serial.print(",");
    Serial.print(error); Serial.print(",");
    Serial.print(estado); Serial.print(",");
    Serial.print(tramasOk); Serial.print(",");
    Serial.println(erroresCrc);
  } else {
    Serial.print("ERROR, codigo 0x");
    Serial.println(resultadoLectura, HEX);
  }
}

// ---------------------------------------------------------------------------
// Parser de comandos de texto por el monitor serie (simula a LabVIEW)
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
    Serial.print("[CMD] Formato invalido (falta ':'): ");
    Serial.println(linea);
    return;
  }

  String clave = linea.substring(0, idx);
  String valorStr = linea.substring(idx + 1);
  clave.trim();
  valorStr.trim();
  clave.toUpperCase();

  long valor = valorStr.toInt();

  Serial.print("[CMD] Recibido: ");
  Serial.print(clave);
  Serial.print(" = ");
  Serial.println(valor);

  if (clave == "SP")        escribirHolding(HREG_SETPOINT, valor, "Setpoint");
  else if (clave == "KP")   escribirHolding(HREG_KP,       valor, "Kp");
  else if (clave == "KI")   escribirHolding(HREG_KI,       valor, "Ki");
  else if (clave == "KD")   escribirHolding(HREG_KD,       valor, "Kd");
  else if (clave == "MODE") escribirHolding(HREG_MODE,     valor, "Modo");
  else if (clave == "PWM")  escribirHolding(HREG_PWM_MAN,  valor, "PWM manual");
  else if (clave == "EN")   escribirCoil(COIL_ENABLE, valor != 0, "Enable");
  else if (clave == "EMG")  escribirCoil(COIL_EMG,    valor != 0, "Emergencia");
  else if (clave == "RST")  pulsoResetIntegrador();
  else {
    Serial.print("[CMD] Clave desconocida: ");
    Serial.println(clave);
  }
}

// ---------------------------------------------------------------------------
// Traducciones comando -> Modbus (Sección 4.3 holding registers)
// ---------------------------------------------------------------------------
void escribirHolding(uint16_t direccion, uint16_t valor, const char* nombre) {
  uint8_t resultado = node.writeSingleRegister(direccion, valor);
  Serial.print("  [TX] Holding 0x");
  Serial.print(direccion, HEX);
  Serial.print(" (");
  Serial.print(nombre);
  Serial.print(") = ");
  Serial.print(valor);
  Serial.print(" -> ");
  Serial.println(resultado == node.ku8MBSuccess ? "OK" : "ERROR");
}

// ---------------------------------------------------------------------------
// Traducciones comando -> Modbus (Sección 4.5 coils)
// ---------------------------------------------------------------------------
void escribirCoil(uint16_t direccion, bool estado, const char* nombre) {
  uint8_t resultado = node.writeSingleCoil(direccion, estado ? 1 : 0);
  Serial.print("  [TX] Coil 0x");
  Serial.print(direccion, HEX);
  Serial.print(" (");
  Serial.print(nombre);
  Serial.print(") = ");
  Serial.print(estado ? "ON" : "OFF");
  Serial.print(" -> ");
  Serial.println(resultado == node.ku8MBSuccess ? "OK" : "ERROR");
}

// ---------------------------------------------------------------------------
// RST:1 es un PULSO (Sección 4.5): reinicia el integrador del PID.
// Escribimos el coil en ON y lo devolvemos a OFF poco después, para que
// del lado del esclavo se vea un flanco de subida y no quede "trabado".
// ---------------------------------------------------------------------------
void pulsoResetIntegrador() {
  uint8_t r1 = node.writeSingleCoil(COIL_RESET, 1);
  Serial.print("  [TX] Coil 0x");
  Serial.print(COIL_RESET, HEX);
  Serial.print(" (Reset integrador) = ON -> ");
  Serial.println(r1 == node.ku8MBSuccess ? "OK" : "ERROR");

  delay(50); // pulso breve; ver nota en el mensaje de abajo

  uint8_t r2 = node.writeSingleCoil(COIL_RESET, 0);
  Serial.print("  [TX] Coil 0x");
  Serial.print(COIL_RESET, HEX);
  Serial.print(" (Reset integrador) = OFF -> ");
  Serial.println(r2 == node.ku8MBSuccess ? "OK" : "ERROR");
  // Nota: este delay(50) bloquea el loop brevemente. Es aceptable porque
  // RST es un comando manual poco frecuente, pero si mas adelante notan
  // que afecta la cadencia de telemetria, se puede rehacer sin delay
  // usando una maquina de estados no bloqueante.
}
