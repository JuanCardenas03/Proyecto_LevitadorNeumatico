/*
  ESP32 - Envío periódico de tramas de telemetría simuladas
  Formato: DATA:sp,altura,pwm,error,estado,tramas_ok,errores_crc\n

  Ejemplo real: DATA:500,226,149,74,3,1520,2\n

  Rangos:
    sp           [0 - 500]
    altura       [0   - 500]
    pwm          [0   - 1000]
    error        [0   - 10]
    estado       [0   - 5]
    tramas_ok    [0   - 10000]  (contador acumulativo, con reinicio al llegar al máximo)
    errores_crc  [0   - 10000]  (contador acumulativo, con reinicio al llegar al máximo)
*/

#define BAUD_RATE 115200
#define INTERVALO_ENVIO_MS 1000  // cada cuánto se envía una trama

// Variables simuladas
int sp = 500;
int altura = 0;
int pwm = 0;
int error_val = 0;
int estado = 0;

// Contadores de comunicación
uint32_t tramas_ok = 0;
uint32_t errores_crc = 0;

unsigned long ultimoEnvio = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  randomSeed(analogRead(0)); // semilla aleatoria basada en ruido del ADC

  sp = random(0, 501); // valor inicial de SP
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - ultimoEnvio >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = ahora;

    actualizarSimulacion();
    enviarTrama();
  }

  // Aquí se podría procesar la recepción de comandos (START, STOP, PID, etc.)
  // si el proyecto los sigue usando en paralelo.
}

// Actualiza los valores simulados de forma "realista"
void actualizarSimulacion() {
  // Altura se acerca gradualmente al SP (simulación simple tipo primer orden)
  int diferencia = sp - altura;
  altura += diferencia * 0.1;
  altura = constrain(altura, 0, 500);

  // Error = diferencia entre SP y altura, limitado al rango permitido
  error_val = constrain(abs(diferencia) % 11, 0, 10);

  // PWM proporcional al error, con algo de variación
  pwm = constrain(map(abs(diferencia), 0, 1000, 0, 1000) + random(-20, 20), 0, 1000);

  // Estado simulado (0 a 5): por ejemplo 0=IDLE,1=INIT,2=RUN,3=ESTABLE,4=AJUSTE,5=ERROR
  if (abs(diferencia) < 5) {
    estado = 3; // estable
  } else if (abs(diferencia) < 50) {
    estado = 2; // en ejecución
  } else {
    estado = 1; // ajustando
  }

  // Contador de tramas correctas: se incrementa cada envío
  tramas_ok++;
  if (tramas_ok > 10000) tramas_ok = 0;

  // Simulación ocasional de error de CRC (aprox 1 de cada 50 tramas)
  if (random(0, 50) == 0) {
    errores_crc++;
    if (errores_crc > 10000) errores_crc = 0;
  }

  // Cada cierto tiempo se cambia el SP para simular nuevas consignas (opcional)
  if (random(0, 200) == 0) {
    sp = random(0, 501);
  }
}

// Envía la trama con el formato exacto requerido
void enviarTrama() {
  Serial.print("DATA");
  Serial.print(",");
  Serial.print(sp);
  Serial.print(",");
  Serial.print(altura);
  Serial.print(",");
  Serial.print(pwm);
  Serial.print(",");
  Serial.print(error_val);
  Serial.print(",");
  Serial.print(estado);
  Serial.print(",");
  Serial.print(tramas_ok);
  Serial.print(",");
  Serial.print(errores_crc);
  Serial.print("\n");
}
