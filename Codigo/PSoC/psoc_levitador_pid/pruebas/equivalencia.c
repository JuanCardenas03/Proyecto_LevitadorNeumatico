/* Prueba 1: el PID de control.c es el MISMO que el de levitador_esp32_PID.ino
 *
 * ref_*: transcripcion literal de medicion + pidPaso() + entrarPID() del .ino
 * (solo se cambiaron constrain/fabs por funciones C). Se cierra el lazo con
 * la planta simulada usando el duty de la referencia, y en cada paso se
 * alimenta la MISMA distancia a control.c. Se compara el duty paso a paso,
 * con cambios de setpoint y de ganancias en caliente, lecturas perdidas y
 * ecos atipicos. */
#include <stdio.h>
#include <math.h>
#include "control.h"
#include "planta_modelo.h"

/* ================== transcripcion del .ino ================== */
static float constrainf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
#define PERIODO_MS 40
static const float DIST_MIN_CM = 3.0f, SALTO_MAX_CM = 10.0f, ALTURA_TUBO_CM = 42.0f;
static float uFF = 51.0f, Kp = 1.0f, Ki = 0.6f, Kd = 0.2f;
static const float TF_DERIV = 0.12f; static float dutyFloorPID = 0.0f; static const float I_MAX = 40.0f;
static float sp = 0, altura = 0, distanciaBase = 37.0f, dist = 37.0f, dutyPct = 0;
static float I = 0, vFilt = 0, alturaPrev = 0;
static float buf[3] = {37.0f, 37.0f, 37.0f}; static unsigned char idx = 0, fallos = 0;
static void setDuty(float pct) { dutyPct = constrainf(pct, 0.0f, 100.0f); }
static float mediana3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
static void pidPaso(void) {
  const float Ts = PERIODO_MS / 1000.0f;
  float e = sp - altura;
  float af = TF_DERIV / (TF_DERIV + Ts);
  float vRaw = (altura - alturaPrev) / Ts;
  vFilt = af * vFilt + (1.0f - af) * vRaw;
  alturaPrev = altura;
  float Dterm = -Kd * vFilt;
  float uRaw = uFF + Kp * e + I + Dterm;
  float u = constrainf(uRaw, dutyFloorPID, 100.0f);
  int satAlta = (uRaw > 100.0f && e > 0);
  int satBaja = (uRaw < dutyFloorPID && e < 0);
  if (!satAlta && !satBaja) I += Ki * e * Ts;
  I = constrainf(I, -I_MAX, I_MAX);
  setDuty(u);
}
static void ref_medir(float d) {
  int valida = (d >= DIST_MIN_CM) && (d <= distanciaBase + 4.0f) &&
               (fabsf(d - dist) <= SALTO_MAX_CM || fallos >= 3);
  if (valida) { buf[idx] = d; idx = (idx + 1) % 3; fallos = 0; dist = mediana3(buf[0], buf[1], buf[2]); }
  else if (fallos < 255) fallos++;
  altura = constrainf(distanciaBase - dist, 0.0f, ALTURA_TUBO_CM);
}
static void ref_entrarPID(void) { I = 0; vFilt = 0; alturaPrev = altura; }
/* ============================================================ */

int main(void)
{
    mb_data_t mb; planta_t p;
    int k, n = 0; double maxdif = 0; float d; uint16_t permil;
    double maxdif_permil = 0;

    mb_init(&mb); ctrl_init(); planta_init(&p, 37.0f, 777u);
    mb.coil[COIL_EN] = 1; mb.hr[HR_MODO] = 1; mb.hr[HR_SETPOINT] = 150; sp = 15.0f;
    ref_entrarPID();

    for (k = 0; k < 25 * 120; k++) {               /* 120 s */
        if (k == 25 * 30) { mb.hr[HR_SETPOINT] = 250; sp = 25.0f; }
        if (k == 25 * 55) { mb.hr[HR_SETPOINT] = 60;  sp = 6.0f; }
        if (k == 25 * 70) { mb.hr[HR_KP] = 180; Kp = 1.8f; mb.hr[HR_KD] = 0; Kd = 0.0f; }
        if (k == 25 * 85) { mb.hr[HR_SETPOINT] = 320; sp = 32.0f; mb.hr[HR_KI] = 150; Ki = 1.5f; }
        if (k == 25 * 100){ mb.hr[HR_SETPOINT] = 0; sp = 0.0f; }   /* fuerza saturacion baja */

        d = planta_medir(&p);
        ref_medir(d);
        pidPaso();
        permil = ctrl_paso(&mb, d, 0);

        double dif = fabs((double)dutyPct - (double)ctrl_info()->duty);
        if (dif > maxdif) maxdif = dif;
        double dp = fabs((double)permil - (double)lroundf(dutyPct * 10.0f));
        if (dp > maxdif_permil) maxdif_permil = dp;
        if (fabsf(I - ctrl_info()->I) > 1e-5f) n++;

        planta_avanzar(&p, dutyPct, 40);
    }
    printf("Pasos comparados: %d (120 s)\n", k);
    printf("Max |duty_ino - duty_psoc| = %.8f %%\n", maxdif);
    printf("Max diferencia en por mil   = %.0f\n", maxdif_permil);
    printf("Pasos con integrador distinto: %d\n", n);
    printf("%s\n", (maxdif < 1e-4 && n == 0) ? "EQUIVALENCIA: PASA" : "EQUIVALENCIA: FALLA");
    return (maxdif < 1e-4 && n == 0) ? 0 : 1;
}
