/*
 * ============================================================================
 *  Levitador Neumático - Firmware ESP32 (maestro Modbus RTU)
 *  FASE 1: Prueba de la ESP32 sola, contra PC (adaptador USB-RS485) como esclavo
 *  Librería: ModbusMaster (Doc Walker) v2.0.1
 * ============================================================================
 *
 *  Objetivo de este sketch:
 *   - Validar la cadena Modbus completa (DE/RE, UART, CRC) SIN depender de
 *     la PSoC. El "esclavo" en esta fase es tu PC corriendo QModMaster (o
 *     similar) en modo esclavo, dirección 1, conectado vía el adaptador
 *     USB-RS485.
 *   - Escribe periódicamente un setpoint de prueba (holding 0x0000) y lee
 *     los 6 input registers de telemetría (0x0000..0x0005), imprimiendo todo
 *     por el monitor serie USB para depurar.
 *
 *  Conexiones (según acuerdo del equipo):
 *   DI  (MAX485) -> GPIO17 (TX2)
 *   RO  (MAX485) -> GPIO16 (RX2)
 *   DE+RE unidos -> GPIO4
 *
 *  Se usa Serial2 (UART2) para el bus RS485, dejando Serial (USB, UART0)
 *  libre para depuración por monitor serie. Esto es necesario porque en
 *  fases posteriores el USB se usará para hablar con LabVIEW y el bus RS485
 *  debe seguir funcionando en paralelo por otro UART.
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

// Marca de tiempo del último ciclo de prueba
unsigned long ultimoCiclo = 0;
const unsigned long PERIODO_PRUEBA_MS = 1000; // 1 vez por segundo mientras se valida a mano

// Setpoint de prueba (mm x10). 300 = 30.0 cm, como el ejemplo de la guía.
uint16_t setpointPrueba = 250;

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

  Serial.println("Listo. Verifica en QModMaster (esclavo, dir. 1) que:");
  Serial.println(" - El holding register 0x0000 recibe el setpoint escrito.");
  Serial.println(" - Los input registers 0x0000..0x0005 se pueden leer (aunque");
  Serial.println("   sea con valores simulados en QModMaster).");
  Serial.println();
}

void loop() {
  if (millis() - ultimoCiclo >= PERIODO_PRUEBA_MS) {
    ultimoCiclo = millis();

    // -------------------------------------------------------------
    // 1) Escritura de prueba: setpoint (holding 0x0000)
    // -------------------------------------------------------------
    uint8_t resultadoEscritura = node.writeSingleRegister(HREG_SETPOINT, setpointPrueba);

    Serial.print("[TX] Escribir setpoint = ");
    Serial.print(setpointPrueba);
    Serial.print(" (mm x10) -> ");
    if (resultadoEscritura == node.ku8MBSuccess) {
      Serial.println("OK");
    } else {
      Serial.print("ERROR, codigo 0x");
      Serial.println(resultadoEscritura, HEX);
    }

    // -------------------------------------------------------------
    // 2) Lectura de telemetría: 6 input registers desde 0x0000
    // -------------------------------------------------------------
    uint8_t resultadoLectura = node.readInputRegisters(IREG_ALTURA, IREG_COUNT);

    Serial.print("[RX] Leer telemetria (6 regs) -> ");
    if (resultadoLectura == node.ku8MBSuccess) {
      uint16_t altura      = node.getResponseBuffer(IREG_ALTURA);
      uint16_t pwm         = node.getResponseBuffer(IREG_PWM);
      int16_t  error       = (int16_t)node.getResponseBuffer(IREG_ERROR);
      uint16_t estado      = node.getResponseBuffer(IREG_ESTADO);
      uint16_t tramasOk    = node.getResponseBuffer(IREG_TRAMAS_OK);
      uint16_t erroresCrc  = node.getResponseBuffer(IREG_ERRORES_CRC);

      Serial.println("OK");
      Serial.print("   Altura: "); Serial.print(altura / 10.0, 1); Serial.println(" cm");
      Serial.print("   PWM: "); Serial.print(pwm / 10.0, 1); Serial.println(" %");
      Serial.print("   Error: "); Serial.print(error / 10.0, 1); Serial.println(" cm");
      Serial.print("   Estado (bitmap): 0b");
      for (int b = 5; b >= 0; b--) Serial.print((estado >> b) & 0x01);
      Serial.println();
      Serial.print("   Tramas OK: "); Serial.println(tramasOk);
      Serial.print("   Errores CRC: "); Serial.println(erroresCrc);
    } else {
      Serial.print("ERROR, codigo 0x");
      Serial.println(resultadoLectura, HEX);
    }

    Serial.println("---");
  }

  // Nota: en esta fase de prueba no hay nada de LabVIEW que leer todavía,
  // por eso no implementamos aun la lógica de "mantener último setpoint
  // ante timeout de LabVIEW". Eso se añade en la Fase 2, cuando exista
  // un puerto serie hacia el HMI cuyo silencio haya que detectar.
}
