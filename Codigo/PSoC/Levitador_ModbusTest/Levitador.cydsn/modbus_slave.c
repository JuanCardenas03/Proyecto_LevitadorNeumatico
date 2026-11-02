/* ============================================================================
 * modbus_slave.c  -  Esclavo Modbus RTU (logica pura, sin hardware)
 * ==========================================================================*/
#include "modbus_slave.h"
#include <string.h>

/* Rango permitido por holding (Seccion 4.3). Fuera de rango -> excepcion 03 */
static const uint16_t hr_max[MB_NUM_HR] = {
    1000u,   /* Setpoint  cm x 10 (0-100 cm) */
    65535u,  /* Kp x 100 */
    65535u,  /* Ki x 100 */
    65535u,  /* Kd x 100 */
    1u,      /* Modo 0 = manual, 1 = auto */
    1000u    /* PWM manual por mil */
};

uint16_t mb_hr_max(uint16_t dir)
{
    return (dir < MB_NUM_HR) ? hr_max[dir] : 0u;
}

/* CRC-16 Modbus, polinomio 0xA001, valor inicial 0xFFFF */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++) {
        crc ^= buf[i];
        for (b = 0u; b < 8u; b++) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void mb_sync(mb_data_t *d)
{
    d->ir[IR_SETPOINT]  = d->hr[HR_SETPOINT];   /* contrato v2: IR0 = HR0 */
    d->ir[IR_TRAMAS_OK] = d->tramas_ok;
    d->ir[IR_ERR_CRC]   = d->errores_crc;
}

void mb_init(mb_data_t *d)
{
    memset(d, 0, sizeof(*d));
    /* Valores de arranque (los mismos ejemplos de la guia) */
    d->hr[HR_KP]   = 40u;   /* 0.40 */
    d->hr[HR_KI]   = 104u;  /* 1.04 */
    d->hr[HR_KD]   = 22u;   /* 0.22 */
    d->hr[HR_MODO] = 1u;    /* automatico */
    mb_sync(d);
}

/* ---------------------------------------------------------------------------
 * Utilidades
 * -------------------------------------------------------------------------*/
static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

/* Agrega el CRC (byte bajo primero) y devuelve la longitud total */
static uint16_t cerrar(uint8_t *r, uint16_t n)
{
    uint16_t c = mb_crc16(r, n);
    r[n]      = (uint8_t)(c & 0xFFu);
    r[n + 1u] = (uint8_t)(c >> 8);
    return (uint16_t)(n + 2u);
}

static uint16_t excepcion(uint8_t *r, uint8_t fc, uint8_t exc, mb_event_t *ev)
{
    ev->tipo = MB_EV_EXCEPCION;
    ev->exc  = exc;
    r[0] = MB_SLAVE_ID;
    r[1] = (uint8_t)(fc | 0x80u);
    r[2] = exc;
    return cerrar(r, 3u);
}

/* ---------------------------------------------------------------------------
 * FC 03 / FC 04: leer registros
 * -------------------------------------------------------------------------*/
static uint16_t leer_registros(const uint16_t *tabla, uint16_t ntabla,
                               const uint8_t *q, uint16_t len,
                               uint8_t *r, mb_event_t *ev)
{
    uint16_t i, ini, cant;

    if (len != 8u) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    ini  = rd16(&q[2]);
    cant = rd16(&q[4]);
    ev->dir  = ini;
    ev->cant = cant;

    if ((cant < 1u) || (cant > 125u)) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    if (((uint32_t)ini + cant) > ntabla) {
        return excepcion(r, q[1], MB_EXC_DIRECCION, ev);
    }

    r[0] = MB_SLAVE_ID;
    r[1] = q[1];
    r[2] = (uint8_t)(cant * 2u);
    for (i = 0u; i < cant; i++) {
        wr16(&r[3u + 2u * i], tabla[ini + i]);
    }
    return cerrar(r, (uint16_t)(3u + 2u * cant));
}

/* ---------------------------------------------------------------------------
 * FC 01: leer coils
 * -------------------------------------------------------------------------*/
static uint16_t leer_coils(const mb_data_t *d, const uint8_t *q, uint16_t len,
                           uint8_t *r, mb_event_t *ev)
{
    uint16_t i, ini, cant, nbytes;

    if (len != 8u) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    ini  = rd16(&q[2]);
    cant = rd16(&q[4]);
    ev->dir  = ini;
    ev->cant = cant;

    if ((cant < 1u) || (cant > 2000u)) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    if (((uint32_t)ini + cant) > MB_NUM_COILS) {
        return excepcion(r, q[1], MB_EXC_DIRECCION, ev);
    }

    nbytes = (uint16_t)((cant + 7u) / 8u);
    r[0] = MB_SLAVE_ID;
    r[1] = q[1];
    r[2] = (uint8_t)nbytes;
    memset(&r[3], 0, nbytes);
    for (i = 0u; i < cant; i++) {
        if (d->coil[ini + i] != 0u) {
            r[3u + i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
    return cerrar(r, (uint16_t)(3u + nbytes));
}

/* ---------------------------------------------------------------------------
 * FC 05: escribir un coil
 * -------------------------------------------------------------------------*/
static uint16_t escribir_coil(mb_data_t *d, const uint8_t *q, uint16_t len,
                              uint8_t *r, mb_event_t *ev)
{
    uint16_t dir, val;

    ev->es_escritura = 1u;
    if (len != 8u) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    dir = rd16(&q[2]);
    val = rd16(&q[4]);
    ev->dir   = dir;
    ev->cant  = 1u;
    ev->valor = val;

    if ((val != 0xFF00u) && (val != 0x0000u)) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    if (dir >= MB_NUM_COILS) {
        return excepcion(r, q[1], MB_EXC_DIRECCION, ev);
    }

    d->coil[dir] = (val == 0xFF00u) ? 1u : 0u;

    memcpy(r, q, 6u);           /* respuesta = eco de la peticion */
    r[0] = MB_SLAVE_ID;
    return cerrar(r, 6u);
}

/* ---------------------------------------------------------------------------
 * FC 06: escribir un holding
 * -------------------------------------------------------------------------*/
static uint16_t escribir_registro(mb_data_t *d, const uint8_t *q, uint16_t len,
                                  uint8_t *r, mb_event_t *ev)
{
    uint16_t dir, val;

    ev->es_escritura = 1u;
    if (len != 8u) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    dir = rd16(&q[2]);
    val = rd16(&q[4]);
    ev->dir   = dir;
    ev->cant  = 1u;
    ev->valor = val;

    if (dir >= MB_NUM_HR) {
        return excepcion(r, q[1], MB_EXC_DIRECCION, ev);
    }
    if (val > hr_max[dir]) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }

    d->hr[dir] = val;

    memcpy(r, q, 6u);
    r[0] = MB_SLAVE_ID;
    return cerrar(r, 6u);
}

/* ---------------------------------------------------------------------------
 * FC 16: escribir varios holding (atomico: o se escriben todos o ninguno)
 * -------------------------------------------------------------------------*/
static uint16_t escribir_varios(mb_data_t *d, const uint8_t *q, uint16_t len,
                                uint8_t *r, mb_event_t *ev)
{
    uint16_t i, ini, cant, v;
    uint8_t  nbytes;

    ev->es_escritura = 1u;
    if (len < 9u) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    ini    = rd16(&q[2]);
    cant   = rd16(&q[4]);
    nbytes = q[6];
    ev->dir  = ini;
    ev->cant = cant;

    if ((cant < 1u) || (cant > 123u) || (nbytes != (cant * 2u)) ||
        (len != (uint16_t)(9u + nbytes))) {
        return excepcion(r, q[1], MB_EXC_VALOR, ev);
    }
    if (((uint32_t)ini + cant) > MB_NUM_HR) {
        return excepcion(r, q[1], MB_EXC_DIRECCION, ev);
    }
    for (i = 0u; i < cant; i++) {
        v = rd16(&q[7u + 2u * i]);
        if (v > hr_max[ini + i]) {
            ev->valor = v;
            return excepcion(r, q[1], MB_EXC_VALOR, ev);
        }
    }
    for (i = 0u; i < cant; i++) {
        d->hr[ini + i] = rd16(&q[7u + 2u * i]);
    }
    ev->valor = d->hr[ini];

    memcpy(r, q, 6u);           /* id, fc, inicio, cantidad */
    r[0] = MB_SLAVE_ID;
    return cerrar(r, 6u);
}

/* ---------------------------------------------------------------------------
 * Punto de entrada
 * -------------------------------------------------------------------------*/
uint16_t mb_process(mb_data_t *d, const uint8_t *req, uint16_t len,
                    uint8_t *resp, mb_event_t *ev)
{
    uint16_t n;

    memset(ev, 0, sizeof(*ev));

    if (len < 4u) {
        ev->tipo = MB_EV_CORTA;
        d->errores_crc++;
        mb_sync(d);
        return 0u;
    }

    ev->id       = req[0];
    ev->fc       = req[1];
    ev->crc_rx   = (uint16_t)(req[len - 2u] | ((uint16_t)req[len - 1u] << 8));
    ev->crc_calc = mb_crc16(req, (uint16_t)(len - 2u));

    if (ev->crc_rx != ev->crc_calc) {
        ev->tipo = MB_EV_CRC_MALO;
        d->errores_crc++;
        mb_sync(d);
        return 0u;
    }

    if ((req[0] != MB_SLAVE_ID) && (req[0] != 0u)) {
        ev->tipo = MB_EV_OTRO_ID;
        return 0u;
    }

    ev->broadcast = (req[0] == 0u) ? 1u : 0u;
    ev->tipo      = MB_EV_OK;
    d->tramas_ok++;
    mb_sync(d);   /* la respuesta ya lleva el contador actualizado */

    switch (req[1]) {
    case 0x01u: n = leer_coils(d, req, len, resp, ev);                     break;
    case 0x03u: n = leer_registros(d->hr, MB_NUM_HR, req, len, resp, ev);  break;
    case 0x04u: n = leer_registros(d->ir, MB_NUM_IR, req, len, resp, ev);  break;
    case 0x05u: n = escribir_coil(d, req, len, resp, ev);                  break;
    case 0x06u: n = escribir_registro(d, req, len, resp, ev);              break;
    case 0x10u: n = escribir_varios(d, req, len, resp, ev);                break;
    default:    n = excepcion(resp, req[1], MB_EXC_FUNCION, ev);           break;
    }

    mb_sync(d);   /* si se escribio HR0, IR0 lo refleja de inmediato */

    return (ev->broadcast != 0u) ? 0u : n;   /* broadcast: nunca se responde */
}
