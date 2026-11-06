/* ============================================================================
 * planta_modelo.c  -  Levitador simulado
 * ==========================================================================*/
#include "planta_modelo.h"

#define G_CM_S2     981.0f   /* gravedad                                   */
#define VT_CM_S     180.0f   /* velocidad terminal de la pelota            */
#define K_AIRE      7.0f     /* cm/s de aire por % de duty sobre la zona muerta */
#define ZONA_MUERTA 20.0f    /* % por debajo del cual el ventilador no sopla */
#define BETA        0.015f   /* perdida de velocidad del aire por cm de altura */
#define TAU_FAN     0.20f    /* s, inercia del ventilador                  */
#define TECHO_CM    3.0f     /* la pelota no se acerca mas que esto al sensor */

#define P_SIN_ECO   0.010f   /* probabilidad de lectura perdida            */
#define P_ATIPICO   0.015f   /* probabilidad de eco de pared               */
#define RUIDO_CM    0.35f    /* escala del ruido (sigma ~ 0.2 cm)          */

static float aleatorio(planta_t *p)   /* 0..1 */
{
    p->semilla = p->semilla * 1664525u + 1013904223u;
    return (float)(p->semilla >> 8) * (1.0f / 16777216.0f);
}

void planta_init(planta_t *p, float base_cm, uint32_t semilla)
{
    p->h       = 0.0f;
    p->v       = 0.0f;
    p->w       = 0.0f;
    p->base    = base_cm;
    p->semilla = (semilla != 0u) ? semilla : 1u;
    p->sin_eco = 0u;
}

void planta_avanzar(planta_t *p, float duty_pct, uint16_t dt_ms)
{
    const float dt = 0.001f;
    float ef, va, rel, a, techo;
    uint16_t i;

    techo = p->base - TECHO_CM;
    for (i = 0u; i < dt_ms; i++) {
        p->w += (duty_pct - p->w) * (dt / TAU_FAN);
        ef = p->w - ZONA_MUERTA;
        if (ef < 0.0f) { ef = 0.0f; }
        va = K_AIRE * ef * (1.0f - BETA * p->h);
        if (va < 0.0f) { va = 0.0f; }

        rel = va - p->v;
        a   = G_CM_S2 * ((rel * ((rel < 0.0f) ? -rel : rel)) / (VT_CM_S * VT_CM_S) - 1.0f);
        p->v += a * dt;
        p->h += p->v * dt;

        if (p->h <= 0.0f)  { p->h = 0.0f;  if (p->v < 0.0f) { p->v = 0.0f; } }
        if (p->h >= techo) { p->h = techo; if (p->v > 0.0f) { p->v = 0.0f; } }
    }
}

float planta_medir(planta_t *p)
{
    float r = aleatorio(p);
    float ruido, d;

    if ((p->sin_eco != 0u) || (r < P_SIN_ECO)) {
        return -1.0f;
    }
    if (r < (P_SIN_ECO + P_ATIPICO)) {
        return 3.0f + aleatorio(p) * (p->base - 3.0f);    /* eco de la pared */
    }
    ruido = (aleatorio(p) + aleatorio(p) + aleatorio(p) + aleatorio(p) - 2.0f) * RUIDO_CM;
    d = p->base - p->h + ruido;
    return (d < 2.0f) ? -1.0f : d;
}
