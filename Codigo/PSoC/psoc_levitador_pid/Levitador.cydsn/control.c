/* ============================================================================
 * control.c  -  Control de ALTURA del levitador (PID + feedforward)
 *
 * Las formulas son las de levitador_esp32_PID.ino (pidPaso, medicion con
 * mediana de 3 y rechazo de atipicos, entrarPID, paro, rampa y barrido).
 * Cambios por estar en la PSoC:
 *   - El setpoint, las ganancias, el modo y el duty manual se leen de los
 *     holding registers; EN, RST y EMG de los coils (Guia v3, Seccion 4).
 *   - La calibracion del fondo no bloquea (se reparte en pasos de 40 ms)
 *     para no dejar de responder Modbus.
 *   - El paro de seguridad del sensor pone EN = 0 (hay que volver a
 *     habilitar desde el HMI, igual que en la ESP32 habia que dar otro sp).
 *   - Manual -> auto: el duty manual inicializa el integrador; uFF no cambia
 *     (en el .ino se adoptaba como uFF).
 *   - Sin math.h: abs y redondeo propios (no hace falta enlazar libm).
 * ==========================================================================*/
#include "control.h"
#include <string.h>

#define TS_S           ((float)CTRL_TS_MS / 1000.0f)
#define CAL_MUESTRAS   15u
#define BANDA_OBJ_X10  5       /* +/- 5 mm (bit 1 del estado) */
#define FALLOS_SENSOR  3u      /* con 3 fallos seguidos el sensor "no esta OK" */
#define FUERA_RANGO_CM 2.0f    /* pelota mas de 2 cm por encima de spMax */

typedef struct {
    ctrl_info_t v;
    float    alturaPrev;
    float    buf[3];
    uint8_t  idx;
    float    t_modo;        /* s transcurridos en rampa o barrido     */
    float    alt0;          /* altura al iniciar la rampa             */
    uint16_t hr0_rampa;     /* ultimo HR0 escrito por la rampa        */
    uint16_t hr0_aviso;     /* ultimo HR0 avisado como recortado      */
    uint8_t  rst_prev;
    uint8_t  pide_rampa;
    uint8_t  pide_barrido;
    uint16_t cal_espera;    /* pasos de espera antes de muestrear     */
    uint16_t cal_i;
    uint16_t cal_n;
    float    cal_suma;
    uint16_t eventos;
} ctrl_t;

static ctrl_t c;

/* ---------------------------------------------------------------------------
 * Utilidades
 * -------------------------------------------------------------------------*/
static float limitar(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

int32_t ctrl_redondear(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static float mediana3(float a, float b, float m)
{
    float t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > m) { t = b; b = m; m = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

const char *ctrl_nombre_modo(ctrl_modo_t m)
{
    switch (m) {
    case CM_PARO:       return "PARO";
    case CM_PID:        return "PID";
    case CM_RAMPA:      return "RAMPA";
    case CM_BARRIDO:    return "BARRIDO";
    case CM_MANUAL:     return "MANUAL";
    case CM_CALIBRANDO: return "CALIBRANDO";
    default:            return "?";
    }
}

/* Fija la distancia de reposo y reinicia el filtro (como calibrar() del .ino) */
static void fijar_base(float base)
{
    c.v.base   = base;
    c.v.spMax  = limitar(base - CTRL_MARGEN_SUP_CM, 5.0f, CTRL_ALTURA_TUBO_CM);
    c.v.dist   = base;
    c.buf[0]   = base;
    c.buf[1]   = base;
    c.buf[2]   = base;
    c.v.altura = 0.0f;
    c.alturaPrev = 0.0f;
}

/* paro() del .ino: ventilador apagado, integrador y derivada a cero */
static void paro(void)
{
    c.v.modo     = CM_PARO;
    c.v.duty     = 0.0f;
    c.v.I        = 0.0f;
    c.v.vFilt    = 0.0f;
    c.v.uRaw     = 0.0f;
    c.v.saturado = 0u;
    c.pide_rampa   = 0u;
    c.pide_barrido = 0u;
}

/* entrarPID() del .ino: ingreso sin salto. Si venia de manual, el integrador
 * arranca en (duty manual - uFF), asi la primera salida es el duty manual.
 * uFF NO se toca: solo cambia con el comando 'ff' de la consola. */
static void entrar_pid(void)
{
    c.v.I = 0.0f;
    if ((c.v.modo == CM_MANUAL) && (c.v.duty > 1.0f)) {
        c.v.I = limitar(c.v.duty - c.v.uFF, -CTRL_I_MAX, CTRL_I_MAX);
        c.eventos |= CTRL_EV_SIN_SALTO;
    }
    c.v.vFilt    = 0.0f;
    c.alturaPrev = c.v.altura;
    c.v.modo     = CM_PID;
}

/* ---------------------------------------------------------------------------
 * Inicializacion y API
 * -------------------------------------------------------------------------*/
void ctrl_init(void)
{
    memset(&c, 0, sizeof(c));
    c.v.uFF  = CTRL_UFF_DEFECTO;
    c.v.piso = CTRL_PISO_DEFECTO;
    fijar_base(CTRL_BASE_DEFECTO_CM);
    c.hr0_aviso = 0xFFFFu;
    paro();
}

const ctrl_info_t *ctrl_info(void)
{
    return &c.v;
}

uint16_t ctrl_eventos(void)
{
    uint16_t e = c.eventos;
    c.eventos = 0u;
    return e;
}

void ctrl_calibrar(uint16_t espera_ms)
{
    paro();
    c.v.modo     = CM_CALIBRANDO;
    c.cal_espera = (uint16_t)(espera_ms / CTRL_TS_MS);
    c.cal_i      = 0u;
    c.cal_n      = 0u;
    c.cal_suma   = 0.0f;
}

void ctrl_pedir_rampa(void)    { c.pide_rampa = 1u; }
void ctrl_pedir_barrido(void)  { c.pide_barrido = 1u; }
void ctrl_set_uff(float pct)   { c.v.uFF  = limitar(pct, 0.0f, 100.0f); }
void ctrl_set_piso(float pct)  { c.v.piso = limitar(pct, 0.0f, 90.0f); }

void ctrl_reset_integrador(void)
{
    c.v.I = 0.0f;
    c.eventos |= CTRL_EV_RST_INT;
}

/* ---------------------------------------------------------------------------
 * Calibracion del fondo (calibrarBase del .ino, repartida en pasos)
 * -------------------------------------------------------------------------*/
static void paso_calibracion(float d)
{
    c.v.duty = 0.0f;
    if (c.cal_espera > 0u) {
        c.cal_espera--;
        return;
    }
    if ((d >= CTRL_DIST_MIN_CM) && (d <= (CTRL_ALTURA_TUBO_CM + 10.0f))) {
        c.cal_suma += d;
        c.cal_n++;
    }
    c.cal_i++;
    if (c.cal_i >= CAL_MUESTRAS) {
        if (c.cal_n >= 5u) {
            fijar_base(c.cal_suma / (float)c.cal_n);
            c.eventos |= CTRL_EV_CAL_OK;
        } else {
            fijar_base(c.v.base);          /* conserva la base anterior */
            c.eventos |= CTRL_EV_CAL_FALLA;
        }
        c.v.fallos = 0u;
        paro();
    }
}

/* ---------------------------------------------------------------------------
 * Medicion con rechazo de atipicos (loop() del .ino)
 * -------------------------------------------------------------------------*/
static void medir(float d)
{
    uint8_t valida = (d >= CTRL_DIST_MIN_CM) && (d <= (c.v.base + 4.0f)) &&
                     ((absf(d - c.v.dist) <= CTRL_SALTO_MAX_CM) || (c.v.fallos >= 3u));

    if (valida) {
        c.buf[c.idx] = d;
        c.idx = (uint8_t)((c.idx + 1u) % 3u);
        c.v.fallos = 0u;
        c.v.dist = mediana3(c.buf[0], c.buf[1], c.buf[2]);
    } else if (c.v.fallos < 255u) {
        c.v.fallos++;                       /* conserva la ultima distancia buena */
    }
    c.v.altura = limitar(c.v.base - c.v.dist, 0.0f, CTRL_ALTURA_TUBO_CM);
}

/* ---------------------------------------------------------------------------
 * PID (pidPaso del .ino, linea por linea)
 * -------------------------------------------------------------------------*/
static void pid_paso(float Kp, float Ki, float Kd)
{
    const float Ts = TS_S;
    float e = c.v.sp - c.v.altura;                 /* cm (positivo = subir) */

    /* Velocidad filtrada para la D. Derivada sobre la medida. */
    float af   = CTRL_TF_DERIV_S / (CTRL_TF_DERIV_S + Ts);
    float vRaw = (c.v.altura - c.alturaPrev) / Ts; /* cm/s */
    float Dterm, uRaw, u;
    uint8_t satAlta, satBaja;

    c.v.vFilt    = af * c.v.vFilt + (1.0f - af) * vRaw;
    c.alturaPrev = c.v.altura;
    Dterm        = -Kd * c.v.vFilt;                /* se opone al movimiento */

    /* Salida: feedforward de sostenimiento + PI + D */
    uRaw = c.v.uFF + Kp * e + c.v.I + Dterm;
    u    = limitar(uRaw, c.v.piso, 100.0f);

    /* Anti-windup: no integra si esta saturado y el error empuja a mas saturacion */
    satAlta = (uRaw > 100.0f) && (e > 0.0f);
    satBaja = (uRaw < c.v.piso) && (e < 0.0f);
    if (!satAlta && !satBaja) {
        c.v.I += Ki * e * Ts;
    }
    c.v.I = limitar(c.v.I, -CTRL_I_MAX, CTRL_I_MAX);

    c.v.uRaw     = uRaw;
    c.v.saturado = ((uRaw > 100.0f) || (uRaw < c.v.piso)) ? 1u : 0u;
    c.v.duty     = u;
}

/* ---------------------------------------------------------------------------
 * Telemetria: IR1..IR4 (IR0, IR5 e IR6 los pone mb_sync)
 * -------------------------------------------------------------------------*/
static void actualizar_ir(mb_data_t *d, uint8_t enlace_caido, uint8_t pid_activo)
{
    int32_t  alt10 = ctrl_redondear(c.v.altura * 10.0f);
    int32_t  err   = (int32_t)d->hr[HR_SETPOINT] - alt10;   /* = IR0 - IR1 */
    int32_t  e_pid = ctrl_redondear(c.v.sp * 10.0f) - alt10;
    uint16_t est   = 0u;

    if ((c.v.modo != CM_CALIBRANDO) && (c.v.fallos < FALLOS_SENSOR)) {
        est |= EST_SENSOR_OK;
    }
    if (pid_activo) {
        if ((e_pid <= BANDA_OBJ_X10) && (e_pid >= -BANDA_OBJ_X10) &&
            ((est & EST_SENSOR_OK) != 0u))                        { est |= EST_EN_OBJETIVO; }
        if (c.v.saturado != 0u)                                   { est |= EST_PWM_SAT; }
        if (absf(c.v.I) >= CTRL_I_MAX)                            { est |= EST_INT_SAT; }
    }
    if (enlace_caido != 0u)                                       { est |= EST_TIMEOUT_COM; }
    if (c.v.altura > (c.v.spMax + FUERA_RANGO_CM))                { est |= EST_FUERA_RANGO; }

    d->ir[IR_ALTURA] = (uint16_t)alt10;
    d->ir[IR_PWM]    = (uint16_t)ctrl_redondear(limitar(c.v.duty, 0.0f, 100.0f) * 10.0f);
    d->ir[IR_ERROR]  = (uint16_t)(int16_t)err;     /* int16 en complemento a 2 */
    d->ir[IR_ESTADO] = est;
    mb_sync(d);
}

/* ---------------------------------------------------------------------------
 * Paso de control
 * -------------------------------------------------------------------------*/
uint16_t ctrl_paso(mb_data_t *d, float dist_cm, uint8_t enlace_caido)
{
    uint8_t  habil;
    uint8_t  pid_activo = 0u;
    uint16_t hr0;

    c.v.dist_cruda = dist_cm;

    if (c.v.modo == CM_CALIBRANDO) {
        paso_calibracion(dist_cm);
    } else {
        medir(dist_cm);

        /* Reset del integrador: flanco de subida del coil 1 (pulso de 50 ms) */
        if ((d->coil[COIL_RST_INT] != 0u) && (c.rst_prev == 0u)) {
            c.v.I = 0.0f;
            c.v.pulsos_rst++;
            c.eventos |= CTRL_EV_RST_INT;
        }

        /* Paro de seguridad: el sensor no responde y el lazo esta cerrado */
        if ((c.v.fallos >= CTRL_MAX_FALLOS) &&
            ((c.v.modo == CM_PID) || (c.v.modo == CM_RAMPA)) &&
            (d->coil[COIL_EN] != 0u)) {
            d->coil[COIL_EN] = 0u;
            c.eventos |= CTRL_EV_SENSOR_PERDIDO;
        }

        habil = (d->coil[COIL_EN] != 0u) && (d->coil[COIL_EMG] == 0u);

        if (!habil) {
            paro();
        } else if ((c.pide_barrido != 0u) || (c.v.modo == CM_BARRIDO)) {
            /* Barrido PWM 0 -> 100 % en lazo abierto (caracterizacion) */
            if (c.pide_barrido != 0u) {
                c.pide_barrido = 0u;
                c.pide_rampa   = 0u;
                c.v.modo       = CM_BARRIDO;
                c.v.I          = 0.0f;
                c.t_modo       = 0.0f;
            }
            c.t_modo += TS_S;
            if (c.t_modo >= CTRL_T_RAMPA_S) {
                paro();
                d->coil[COIL_EN] = 0u;
                c.eventos |= CTRL_EV_FIN_BARRIDO;
            } else {
                c.v.duty = 100.0f * c.t_modo / CTRL_T_RAMPA_S;
            }
        } else if (d->hr[HR_MODO] == 0u) {
            /* MANUAL: duty = HR5 (por mil) */
            c.pide_rampa = 0u;
            c.v.modo     = CM_MANUAL;
            c.v.duty     = (float)d->hr[HR_PWM_MANUAL] / 10.0f;
            c.v.saturado = 0u;
        } else {
            /* AUTOMATICO */
            if ((c.v.modo != CM_PID) && (c.v.modo != CM_RAMPA)) {
                entrar_pid();
            }
            if (c.pide_rampa != 0u) {
                c.pide_rampa = 0u;
                c.v.modo     = CM_RAMPA;
                c.t_modo     = 0.0f;
                c.alt0       = c.v.altura;
                c.hr0_rampa  = d->hr[HR_SETPOINT];
            }

            if (c.v.modo == CM_RAMPA) {
                if (d->hr[HR_SETPOINT] != c.hr0_rampa) {
                    c.v.modo = CM_PID;                 /* llego un SP nuevo */
                    c.eventos |= CTRL_EV_RAMPA_CANCEL;
                } else {
                    c.t_modo += TS_S;
                    if (c.t_modo >= CTRL_T_RAMPA_S) {
                        c.v.sp   = c.v.spMax;
                        c.v.modo = CM_PID;
                        c.eventos |= CTRL_EV_FIN_RAMPA;
                    } else {
                        c.v.sp = c.alt0 + (c.v.spMax - c.alt0) * c.t_modo / CTRL_T_RAMPA_S;
                    }
                    /* el HMI ve el setpoint de la rampa (IR0 = HR0) */
                    c.hr0_rampa = (uint16_t)ctrl_redondear(c.v.sp * 10.0f);
                    d->hr[HR_SETPOINT] = c.hr0_rampa;
                    c.hr0_aviso = c.hr0_rampa;
                }
            }

            if (c.v.modo == CM_PID) {
                hr0    = d->hr[HR_SETPOINT];
                c.v.sp = (float)hr0 / 10.0f;
                if (c.v.sp > c.v.spMax) {
                    c.v.sp = c.v.spMax;
                    if (hr0 != c.hr0_aviso) {
                        c.eventos |= CTRL_EV_SP_RECORTADO;
                    }
                }
                c.hr0_aviso = hr0;
            }

            pid_paso((float)d->hr[HR_KP] / 100.0f,
                     (float)d->hr[HR_KI] / 100.0f,
                     (float)d->hr[HR_KD] / 100.0f);
            pid_activo = 1u;
        }
    }

    c.rst_prev = d->coil[COIL_RST_INT];
    actualizar_ir(d, enlace_caido, pid_activo);

    return (uint16_t)ctrl_redondear(limitar(c.v.duty, 0.0f, 100.0f) * 10.0f);
}