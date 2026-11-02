/*
  Levitador neumatico - Control de ALTURA con ESP32 (Arduino IDE)
  ================================================================

  Hardware (igual al de lazo abierto):
   - PWM -> IRLZ44N (lado bajo) -> ventilador 24V YXD9225HB24R
   - HC-SR04 arriba del tubo apuntando hacia abajo (ECHO por divisor 1k/2k)
   - Tubo de 42 cm, pelota (icopor + cartones) de 5.5 cm, fan a 7 cm del piso

  Cambios de fondo respecto al codigo anterior:
   1) FEEDFORWARD (uFF): un duty de sostenimiento que mantiene la pelota flotando.
      El PID solo corrige ALREDEDOR de ese punto -> mucho mas estable y rapido.
   2) Control PI por defecto (Kd=0). Los cartones dan harto amortiguamiento
      aerodinamico, asi que casi siempre PI + feedforward basta.
      La D es opcional y actua como amortiguador de velocidad (filtrada).
   3) Filtro + rechazo de lecturas atipicas del HC-SR04 (ecos de la pared).
   4) Anti-windup e ingreso al PID sin salto (bumpless).
   5) Geometria real (tubo 42 cm).

  Monitor Serie a 115200, con "Nueva linea". Comandos al final (o escribe '?').
  Salida para Serial Plotter:  sp, altura, pwm
*/

// ================== Pines ==================
const int PIN_PWM  = 25;   // -> 220 ohm -> Gate del MOSFET
const int PIN_TRIG = 5;    // -> TRIG del HC-SR04
const int PIN_ECHO = 18;   // <- ECHO del HC-SR04 (POR EL DIVISOR 1k/2k)

// ================== PWM ==================
const uint32_t PWM_FREQ = 25000;   // Hz
const uint8_t  PWM_RES  = 10;      // 0..1023
const uint8_t  PWM_CH   = 0;       // solo core 2.x
const uint32_t PWM_MAX  = (1UL << PWM_RES) - 1;

// ================== Sensor / tiempos ==================
const uint32_t PERIODO_MS      = 40;     // Ts del PID = 40 ms (25 Hz). Mas rapido que antes -> menos retardo
const uint32_t ECHO_TIMEOUT_US = 12000;  // ~2 m
const float    VEL_SONIDO      = 0.0343; // cm/us
const float    DIST_MIN_CM     = 3.0;    // el HC-SR04 no mide bien por debajo
const float    SALTO_MAX_CM    = 10.0;   // rechazo de saltos (ecos de pared); 10 cm en 40 ms = 2.5 m/s
const uint8_t  MAX_FALLOS      = 20;      // ~0.8 s sin eco -> paro de seguridad

// ================== Tubo ==================
const float ALTURA_TUBO_CM  = 42.0;
const float MARGEN_SUP_CM   = 5.0;   // la pelota no sube mas cerca que esto del sensor
const float BASE_DEFECTO_CM = 37.0;  // se usa si falla la calibracion

// ================== Rampa y barrido ==================
const float T_RAMPA_S = 30.0;

// ================== CONTROL ==================
// ---- Punto de sostenimiento (FEEDFORWARD) ----
//   uFF = duty (%) que, a lazo abierto, hace flotar la pelota hacia la mitad del tubo.
//   OBLIGATORIO calibrarlo: usa 'u=40', 'u=50', ... hasta que flote, y pon ese valor
//   en ff=  (o al pasar de manual a PID se adopta el duty manual automaticamente).
float uFF = 51.0;          // % (PLACEHOLDER: calibrar con modo manual)

// ---- Ganancias (puntos de ARRANQUE, sintonizar segun el procedimiento) ----
float Kp = 1.0;   // % de duty por cm de error
float Ki = 0.6;   // % de duty por (cm*s)
float Kd = 0.2;   // % de duty por (cm/s)  -> amortiguador; dejar 0 al inicio

const float TF_DERIV   = 0.12;   // s, filtro de la velocidad (para la D)
float dutyFloorPID = 0.0;        // % piso de duty en vuelo (sube a ~dead-zone si la pelota se cae feo)
const float I_MAX = 40.0;        // limite del integrador (% alrededor de uFF)

// ================== Estado ==================
enum Modo { PARO, PID, RAMPA, BARRIDO, MANUAL };
Modo modo = PARO;

float sp = 0;                          // cm
float spMax = BASE_DEFECTO_CM - MARGEN_SUP_CM;
float altura = 0;                      // cm desde el fondo
float distanciaBase = BASE_DEFECTO_CM; // cm sensor->pelota en reposo
float dist = BASE_DEFECTO_CM;          // distancia filtrada (cm)
bool  verDist = false;
float dutyPct = 0;                     // duty aplicado (%)
float I = 0, vFilt = 0, alturaPrev = 0;
uint32_t tInicio = 0, tUltimaMedida = 0;
float buf[3] = {0, 0, 0};
uint8_t idx = 0, fallos = 0;

// ---------- PWM compatible core 2.x / 3.x ----------
void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM, PWM_CH);
#endif
}
void pwmWriteRaw(uint32_t v) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, v);
#else
  ledcWrite(PWM_CH, v);
#endif
}
void setDuty(float pct) {
  dutyPct = constrain(pct, 0.0f, 100.0f);
  pwmWriteRaw((uint32_t)(dutyPct / 100.0f * PWM_MAX + 0.5f));
}

// ---------- HC-SR04 ----------
float medirCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long t = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (t == 0) return -1;                  // sin eco
  return t * VEL_SONIDO / 2.0f;
}
float mediana3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
bool calibrarBase() {
  float suma = 0; int n = 0;
  for (int i = 0; i < 15; i++) {
    float d = medirCm();
    if (d >= DIST_MIN_CM && d <= ALTURA_TUBO_CM + 10) { suma += d; n++; }
    delay(PERIODO_MS);
  }
  if (n < 5) return false;
  distanciaBase = suma / n;
  return true;
}
void calibrar() {
  Serial.println("# Calibrando fondo (pelota abajo, ventilador apagado)...");
  if (!calibrarBase())
    Serial.printf("# ERROR: HC-SR04 no responde (revisa TRIG/ECHO/divisor). Base = %.1f cm\n", distanciaBase);
  spMax = constrain(distanciaBase - MARGEN_SUP_CM, 5.0f, ALTURA_TUBO_CM);
  dist = distanciaBase;
  for (int i = 0; i < 3; i++) buf[i] = distanciaBase;
  altura = 0; alturaPrev = 0;
  Serial.printf("# Base = %.1f cm | setpoint maximo = %.1f cm\n", distanciaBase, spMax);
}

// ---------- PID ----------
void pidPaso() {
  const float Ts = PERIODO_MS / 1000.0f;
  float e = sp - altura;                         // cm (positivo = hay que subir)

  // Velocidad filtrada para la D (amortiguamiento). Derivada sobre la medida.
  float af = TF_DERIV / (TF_DERIV + Ts);
  float vRaw = (altura - alturaPrev) / Ts;       // cm/s
  vFilt = af * vFilt + (1.0f - af) * vRaw;
  alturaPrev = altura;
  float Dterm = -Kd * vFilt;                     // se opone al movimiento

  // Salida: feedforward de sostenimiento + PI + D
  float uRaw = uFF + Kp * e + I + Dterm;
  float u = constrain(uRaw, dutyFloorPID, 100.0f);

  // Anti-windup: no integra si esta saturado y el error empuja hacia mas saturacion
  bool satAlta = (uRaw > 100.0f      && e > 0);
  bool satBaja = (uRaw < dutyFloorPID && e < 0);
  if (!satAlta && !satBaja) I += Ki * e * Ts;
  I = constrain(I, -I_MAX, I_MAX);

  setDuty(u);
}

// Ingreso al PID sin salto. Si venia de manual, adopta ese duty como sostenimiento.
void entrarPID() {
  if (modo == PID || modo == RAMPA) return;
  if (modo == MANUAL && dutyPct > 1.0f) uFF = dutyPct;
  I = 0; vFilt = 0; alturaPrev = altura;
}
void paro() { modo = PARO; setDuty(0); I = 0; vFilt = 0; }

void fijarSP(float x, bool enPct) {
  float pedido = enPct ? constrain(x, 0.0f, 100.0f) * spMax / 100.0f : x;
  sp = constrain(pedido, 0.0f, spMax);
  entrarPID(); modo = PID;
  if (sp != pedido)
    Serial.printf("# SP %.2f cm FUERA DE RANGO -> %.2f cm (0..%.1f). PID activo (uFF=%.1f%%)\n", pedido, sp, spMax, uFF);
  else
    Serial.printf("# SP = %.2f cm. PID activo (uFF=%.1f%%, Kp=%.2f Ki=%.2f Kd=%.2f)\n", sp, uFF, Kp, Ki, Kd);
}

void ayuda() {
  Serial.println("# Comandos:");
  Serial.println("#   sp=15 | 15 | sp=50%   -> setpoint (cm o % de la altura util) y activa PID");
  Serial.println("#   u=50                  -> MANUAL (lazo abierto) para buscar el duty de flote");
  Serial.println("#   ff=50                 -> fija el feedforward de sostenimiento (%)");
  Serial.println("#   kp= ki= kd=           -> ganancias en caliente");
  Serial.println("#   fl=0                  -> piso de duty en vuelo (%)   | tf no editable aqui");
  Serial.println("#   a  rampa sp->max | b  barrido PWM 0->100 (caracterizar) | p  paro");
  Serial.println("#   c  calibrar fondo | d  ver distancia cruda | ?  ayuda");
  Serial.printf("# Estado: uFF=%.1f%% Kp=%.2f Ki=%.2f Kd=%.2f fl=%.1f%% SP=%.1f cm (max %.1f)\n",
                uFF, Kp, Ki, Kd, dutyFloorPID, sp, spMax);
}

// ---------- Comandos ----------
void leerComandos() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();
  if (cmd.length() == 0) return;

  int eq = cmd.indexOf('=');
  if (eq > 0) {
    String k = cmd.substring(0, eq); String v = cmd.substring(eq + 1);
    k.trim(); v.trim();
    float x = v.toFloat();
    if      (k == "sp") fijarSP(x, v.endsWith("%"));
    else if (k == "kp") { Kp = x < 0 ? 0 : x; Serial.printf("# Kp = %.3f\n", Kp); }
    else if (k == "ki") { Ki = x < 0 ? 0 : x; Serial.printf("# Ki = %.3f\n", Ki); }
    else if (k == "kd") { Kd = x < 0 ? 0 : x; Serial.printf("# Kd = %.3f\n", Kd); }
    else if (k == "ff") { uFF = constrain(x, 0.0f, 100.0f); Serial.printf("# Feedforward uFF = %.1f %%\n", uFF); }
    else if (k == "fl") { dutyFloorPID = constrain(x, 0.0f, 90.0f); Serial.printf("# Piso de duty = %.1f %%\n", dutyFloorPID); }
    else if (k == "u")  { modo = MANUAL; setDuty(x); Serial.printf("# MANUAL. Duty = %.1f %%\n", dutyPct); }
    else ayuda();
    return;
  }

  char c = cmd.charAt(0);
  if (isDigit(c) || c == '.') { fijarSP(cmd.toFloat(), cmd.endsWith("%")); return; }
  switch (c) {
    case 'p': paro(); Serial.println("# PARO"); break;
    case 'a': entrarPID(); modo = RAMPA; sp = altura; tInicio = millis();
              Serial.printf("# RAMPA sp %.1f -> %.1f cm en %.0f s\n", altura, spMax, T_RAMPA_S); break;
    case 'b': modo = BARRIDO; setDuty(0); tInicio = millis();
              Serial.printf("# BARRIDO PWM 0 -> 100%% en %.0f s (caracterizacion)\n", T_RAMPA_S); break;
    case 'd': verDist = !verDist; break;
    case 'c': paro(); delay(1500); calibrar(); break;
    default:  ayuda();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  pwmInit(); setDuty(0);
  delay(500);
  calibrar();
  ayuda();
}

void loop() {
  leerComandos();
  if (millis() - tUltimaMedida < PERIODO_MS) return;
  tUltimaMedida = millis();

  // ---- Medicion con rechazo de atipicos ----
  float d = medirCm();
  bool valida = (d >= DIST_MIN_CM) && (d <= distanciaBase + 4.0f) &&
                (fabs(d - dist) <= SALTO_MAX_CM || fallos >= 3); // tras varios fallos acepta el salto
  if (valida) {
    buf[idx] = d; idx = (idx + 1) % 3; fallos = 0;
    dist = mediana3(buf[0], buf[1], buf[2]);
  } else if (fallos < 255) {
    fallos++;                                   // conserva la ultima distancia buena
  }
  altura = constrain(distanciaBase - dist, 0.0f, ALTURA_TUBO_CM);

  if (fallos == MAX_FALLOS && (modo == PID || modo == RAMPA)) {
    paro(); Serial.println("# PARO DE SEGURIDAD: el sensor no responde");
  }

  // ---- Modos ----
  float t = (millis() - tInicio) / 1000.0f;
  if (modo == RAMPA) {
    if (t >= T_RAMPA_S) { sp = spMax; modo = PID; Serial.println("# Fin de rampa, PID sostiene"); }
    else sp = altura + (spMax - altura) * t / T_RAMPA_S;
  }
  if (modo == PID || modo == RAMPA) pidPaso();
  if (modo == BARRIDO) {
    if (t >= T_RAMPA_S) { paro(); Serial.println("# Fin del barrido"); }
    else setDuty(100.0f * t / T_RAMPA_S);
  }

  // ---- Serial Plotter ----
  Serial.printf("sp:%.2f,altura:%.1f,pwm:%.1f", sp, altura, dutyPct);
  if (verDist) Serial.printf(",dist:%.1f", dist);
  Serial.println();
}
