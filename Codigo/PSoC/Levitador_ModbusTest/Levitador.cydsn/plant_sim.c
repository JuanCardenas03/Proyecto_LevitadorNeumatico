/* ============================================================================
 * plant_sim.c  -  Planta simulada de primer orden
 *
 *  - EN = 0 o EMG = 1 : PWM = 0, la pelota cae.
 *  - Modo auto (HR4=1): la altura va al setpoint (tau 0.8 s); el PWM se
 *                       calcula como equilibrio + ganancia * error.
 *  - Modo manual      : PWM = HR5; la altura sigue una curva PWM -> altura.
 *
 * Es solo para probar la comunicacion; NO es un modelo fisico del levitador.
 * ==========================================================================*/
#include "plant_sim.h"
#include <string.h>

#define PWM_EQUILIBRIO   550.0f   /* PWM con el que "flota" (por mil)     */
#define PWM_GANANCIA     1.5f     /* por mil de PWM por cada mm de error   */
#define PWM_MIN_LEVANTA  400.0f   /* debajo de esto la pelota no sube      */
#define TAU_AUTO         0.8f     /* s */
#define TAU_MANUAL       0.6f     /* s */
#define TAU_CAIDA        0.4f     /* s */
#define BANDA_OBJETIVO   5        /* +/- 5 mm = 5 en cm x 10 (Seccion 4.6) */

static float            alt_sim;   /* cm x 10 */
static float            pwm_sim;   /* por mil */
static plant_override_t ov;
static uint8_t          rst_prev;
static uint16_t         rst_cuenta;

void plant_clear_overrides(void)
{
    memset(&ov, 0, sizeof(ov));
}

plant_override_t *plant_overrides(void)
{
    return &ov;
}

uint16_t plant_resets_integrador(void)
{
    return rst_cuenta;
}

void plant_init(void)
{
    alt_sim    = 0.0f;
    pwm_sim    = 0.0f;
    rst_prev   = 0u;
    rst_cuenta = 0u;
    plant_clear_overrides();
}

static int16_t redondear(float v)
{
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

void plant_step(mb_data_t *d, uint16_t dt_ms, uint8_t enlace_caido)
{
    float    dt  = (float)dt_ms / 1000.0f;
    float    sp  = (float)d->hr[HR_SETPOINT];
    float    objetivo, tau;
    uint8_t  saturado = 0u;
    int16_t  alt, err;
    uint16_t est;

    if ((d->coil[COIL_EMG] != 0u) || (d->coil[COIL_EN] == 0u)) {
        pwm_sim  = 0.0f;
        objetivo = 0.0f;
        tau      = TAU_CAIDA;
    } else if (d->hr[HR_MODO] == 1u) {
        pwm_sim = PWM_EQUILIBRIO + PWM_GANANCIA * (sp - alt_sim);
        if (pwm_sim >= 1000.0f) { pwm_sim = 1000.0f; saturado = 1u; }
        if (pwm_sim <= 0.0f)    { pwm_sim = 0.0f;    saturado = 1u; }
        objetivo = sp;
        tau      = TAU_AUTO;
    } else {
        pwm_sim  = (float)d->hr[HR_PWM_MANUAL];
        objetivo = (pwm_sim <= PWM_MIN_LEVANTA) ? 0.0f :
                   (pwm_sim - PWM_MIN_LEVANTA) * 1000.0f / (1000.0f - PWM_MIN_LEVANTA);
        tau      = TAU_MANUAL;
    }

    /* Pulso de reset del integrador (flanco de subida del coil 1).
     * En la PSoC real aqui se pone el integral del PID en cero. */
    if ((d->coil[COIL_RST_INT] != 0u) && (rst_prev == 0u)) {
        rst_cuenta++;
    }
    rst_prev = d->coil[COIL_RST_INT];

    if (ov.alt_activa != 0u) {
        alt_sim = (float)ov.alt_valor;
    } else {
        alt_sim += (objetivo - alt_sim) * (dt / tau);
    }

    alt = redondear(alt_sim);
    err = (ov.err_activo != 0u) ? ov.err_valor
                                : (int16_t)((int32_t)d->hr[HR_SETPOINT] - alt);

    est = EST_SENSOR_OK;
    if ((err <= BANDA_OBJETIVO) && (err >= -BANDA_OBJETIVO)) { est |= EST_EN_OBJETIVO; }
    if (saturado != 0u)                                     { est |= EST_PWM_SAT; }
    if (enlace_caido != 0u)                                 { est |= EST_TIMEOUT_COM; }
    if ((alt < 0) || (alt > 1000))                          { est |= EST_FUERA_RANGO; }
    if (ov.est_activo != 0u)                                { est = ov.est_valor; }

    d->ir[IR_ALTURA] = (uint16_t)alt;
    d->ir[IR_PWM]    = (uint16_t)redondear(pwm_sim);
    d->ir[IR_ERROR]  = (uint16_t)err;          /* int16 en complemento a 2 */
    d->ir[IR_ESTADO] = est;
}
