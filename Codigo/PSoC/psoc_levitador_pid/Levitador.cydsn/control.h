/* ============================================================================
 * control.h  -  Control de ALTURA del levitador (PID + feedforward)
 *
 * Port directo del PID que mejor funciono en la ESP32 (levitador_esp32_PID.ino):
 *   u = uFF + Kp*e + I - Kd*v_filtrada      (derivada sobre la medida)
 *   - feedforward de sostenimiento uFF
 *   - anti-windup condicional + limite del integrador
 *   - entrada al PID sin salto (si viene de manual, el integrador arranca
 *     en duty_manual - uFF; uFF NO cambia: solo se modifica por consola)
 *   - mediana de 3 + rechazo de saltos del HC-SR04
 *   - paro de seguridad si el sensor deja de responder
 *
 * Modulo SIN hardware: recibe la distancia medida (cm, -1 = sin eco) y
 * devuelve el duty en por mil. Lee hr[]/coil[] y escribe ir[] del mapa
 * Modbus (Guia v3, Seccion 4). Asi se puede probar en el PC tal cual.
 *
 * Unidades de las ganancias (HR1..HR3, x100):  Kp en % de duty por cm,
 * Ki en % por (cm*s), Kd en % por (cm/s). Las mismas del .ino.
 * ==========================================================================*/
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "modbus_slave.h"

#define CTRL_TS_MS            40u      /* periodo del PID: 40 ms (25 Hz)      */

/* Geometria y sensor (mismos valores del .ino) */
#define CTRL_ALTURA_TUBO_CM   42.0f
#define CTRL_MARGEN_SUP_CM     5.0f    /* la pelota no sube mas cerca del sensor */
#define CTRL_BASE_DEFECTO_CM  37.0f    /* si falla la calibracion             */
#define CTRL_DIST_MIN_CM       3.0f    /* el HC-SR04 no mide bien por debajo  */
#define CTRL_SALTO_MAX_CM     10.0f    /* rechazo de saltos (ecos de pared)   */
#define CTRL_MAX_FALLOS       20u      /* ~0.8 s sin eco -> paro de seguridad */

/* Control */
#define CTRL_UFF_DEFECTO      49.0f    /* % feedforward (ajustado en placa)   */
#define CTRL_PISO_DEFECTO     20.0f    /* % piso de duty en vuelo (ajustado)  */
#define CTRL_TF_DERIV_S        0.12f   /* filtro de la velocidad para la D    */
#define CTRL_I_MAX            40.0f    /* limite del integrador (% alrededor de uFF) */
#define CTRL_T_RAMPA_S        30.0f    /* rampa de sp y barrido de PWM        */

/* Ganancias de arranque (x100 en los holding registers), ajustadas en placa */
#define CTRL_KP_DEFECTO_X100  100u     /* Kp = 1.00 */
#define CTRL_KI_DEFECTO_X100   60u     /* Ki = 0.60 */
#define CTRL_KD_DEFECTO_X100    5u     /* Kd = 0.05 */

typedef enum {
    CM_PARO = 0,
    CM_PID,
    CM_RAMPA,
    CM_BARRIDO,
    CM_MANUAL,
    CM_CALIBRANDO
} ctrl_modo_t;

/* Eventos para el log (mascara; ctrl_eventos() los entrega y los borra) */
#define CTRL_EV_CAL_OK         (1u << 0)
#define CTRL_EV_CAL_FALLA      (1u << 1)
#define CTRL_EV_SENSOR_PERDIDO (1u << 2)
#define CTRL_EV_FIN_RAMPA      (1u << 3)
#define CTRL_EV_RAMPA_CANCEL   (1u << 4)
#define CTRL_EV_FIN_BARRIDO    (1u << 5)
#define CTRL_EV_SIN_SALTO      (1u << 6)   /* manual -> auto: I = duty - uFF */
#define CTRL_EV_RST_INT        (1u << 7)
#define CTRL_EV_SP_RECORTADO   (1u << 8)

/* Estado visible (solo lectura desde fuera) */
typedef struct {
    ctrl_modo_t modo;
    float    sp;          /* cm, setpoint que usa el PID                  */
    float    altura;      /* cm desde el fondo                            */
    float    dist;        /* cm sensor->pelota, filtrada                  */
    float    dist_cruda;  /* cm, ultima lectura (-1 = sin eco)            */
    float    duty;        /* % aplicado                                   */
    float    I;           /* % termino integral                           */
    float    vFilt;       /* cm/s velocidad filtrada                      */
    float    uRaw;        /* % salida antes de saturar                    */
    float    uFF;         /* % feedforward de sostenimiento               */
    float    piso;        /* % piso de duty en vuelo                      */
    float    base;        /* cm sensor->pelota en reposo                  */
    float    spMax;       /* cm setpoint maximo                           */
    uint8_t  fallos;      /* lecturas invalidas seguidas                  */
    uint8_t  saturado;    /* 1 = la salida del PID esta saturada          */
    uint16_t pulsos_rst;  /* pulsos de reset del integrador recibidos     */
} ctrl_info_t;

void               ctrl_init(void);

/* Un paso de control (cada CTRL_TS_MS). dist_cm < 0 = lectura fallida.
 * Devuelve el duty en por mil (0..1000) y actualiza ir[1..4]. */
uint16_t           ctrl_paso(mb_data_t *d, float dist_cm, uint8_t enlace_caido);

const ctrl_info_t *ctrl_info(void);
uint16_t           ctrl_eventos(void);
const char        *ctrl_nombre_modo(ctrl_modo_t m);

void               ctrl_calibrar(uint16_t espera_ms);  /* no bloqueante */
void               ctrl_pedir_rampa(void);
void               ctrl_pedir_barrido(void);
void               ctrl_set_uff(float pct);
void               ctrl_set_piso(float pct);
void               ctrl_reset_integrador(void);

int32_t            ctrl_redondear(float v);

#endif /* CONTROL_H */