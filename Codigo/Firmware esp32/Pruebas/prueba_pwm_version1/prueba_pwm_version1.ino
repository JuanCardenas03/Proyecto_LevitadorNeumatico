// --- Configuración del PWM (Versión ESP32 Core 3.x) ---
const int fanPin = 25;       // Pin del ESP32 conectado al MOSFET
const int freq = 25000;      // Frecuencia de 25 kHz
const int resolution = 8;    // Resolución de 8 bits (valores de 0 a 255)
// Nota: En esta versión ya no es necesario definir un canal (pwmChannel)

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando control de ventilador...");

  // 1. Nueva función unificada que configura y asocia el pin
  ledcAttach(fanPin, freq, resolution);

  // 2. Inicializamos el ventilador apagado (Nota: ahora se usa fanPin, no pwmChannel)
  ledcWrite(fanPin, 0);
}

void loop() {
  // Rampa de aceleración de 0 a 255
  for (int dutyCycle = 20; dutyCycle <= 30; dutyCycle++) {
    ledcWrite(fanPin, dutyCycle);
    Serial.print("Velocidad: ");
    Serial.println(dutyCycle);
    delay(3000);
  }

  delay(2000); // Mantiene velocidad máxima por 2 segundos

  // Rampa de desaceleración de 255 a 0
  for (int dutyCycle = 30; dutyCycle >= 20; dutyCycle--) {
    ledcWrite(fanPin, dutyCycle);
    Serial.print("Velocidad: ");
    Serial.println(dutyCycle);
    delay(3000);
  }

  delay(4000); // Mantiene apagado por 2 segundos
}