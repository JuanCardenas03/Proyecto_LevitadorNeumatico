/* ============================================================================
 * debug_log.c  -  Salida legible por UART_DBG
 * ==========================================================================*/
#include "project.h"
#include "debug_log.h"
#include "modbus_slave.h"
#include <stdio.h>
#include <stdarg.h>

#define N_ROT   8u
#define TAM_ROT 24u

static char    rot[N_ROT][TAM_ROT];
static uint8_t rot_i;

static char *siguiente(void)
{
    char *b = rot[rot_i];
    rot_i = (uint8_t)((rot_i + 1u) % N_ROT);
    return b;
}

void dbg_puts(const char *s)
{
    UART_DBG_PutString(s);
}

void dbg_printf(const char *fmt, ...)
{
    char    buf[192];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    UART_DBG_PutString(buf);
}

void dbg_hex(const char *prefijo, const uint8_t *b, uint16_t n)
{
    char     t[4];
    uint16_t i;

    dbg_puts(prefijo);
    for (i = 0u; i < n; i++) {
        if ((i > 0u) && ((i % 32u) == 0u)) {
            dbg_puts("\r\n       ");
        }
        (void)snprintf(t, sizeof(t), "%02X ", b[i]);
        dbg_puts(t);
    }
    dbg_puts("\r\n");
}

const char *fx(int32_t v, uint8_t decimales)
{
    char        *b = siguiente();
    const char  *s = (v < 0) ? "-" : "";
    unsigned long a = (v < 0) ? (unsigned long)(-v) : (unsigned long)v;

    if (decimales == 0u) {
        (void)snprintf(b, TAM_ROT, "%s%lu", s, a);
    } else if (decimales == 1u) {
        (void)snprintf(b, TAM_ROT, "%s%lu.%lu", s, a / 10ul, a % 10ul);
    } else {
        (void)snprintf(b, TAM_ROT, "%s%lu.%02lu", s, a / 100ul, a % 100ul);
    }
    return b;
}

const char *bin6(uint16_t v)
{
    char   *b = siguiente();
    uint8_t i;

    for (i = 0u; i < 6u; i++) {
        b[i] = ((v >> (5u - i)) & 1u) ? '1' : '0';
    }
    b[6] = '\0';
    return b;
}

const char *nombre_hr(uint16_t dir)
{
    static const char *const n[MB_NUM_HR] = {
        "Setpoint", "Kp", "Ki", "Kd", "Modo", "PWM manual"
    };
    return (dir < MB_NUM_HR) ? n[dir] : "?";
}

const char *nombre_ir(uint16_t dir)
{
    static const char *const n[MB_NUM_IR] = {
        "Setpoint vigente", "Altura", "PWM aplicado", "Error",
        "Estado", "Tramas OK", "Errores CRC"
    };
    return (dir < MB_NUM_IR) ? n[dir] : "?";
}

const char *nombre_coil(uint16_t dir)
{
    static const char *const n[MB_NUM_COILS] = {
        "Habilitar control", "Reset integrador", "Paro emergencia"
    };
    return (dir < MB_NUM_COILS) ? n[dir] : "?";
}

const char *nombre_exc(uint8_t exc)
{
    switch (exc) {
    case MB_EXC_FUNCION:   return "funcion no soportada";
    case MB_EXC_DIRECCION: return "direccion invalida";
    case MB_EXC_VALOR:     return "valor fuera de rango / longitud";
    default:               return "?";
    }
}

const char *fmt_hr(uint16_t dir, uint16_t v)
{
    char *b = siguiente();

    switch (dir) {
    case HR_SETPOINT:   (void)snprintf(b, TAM_ROT, "%s cm", fx(v, 1u)); break;
    case HR_KP:
    case HR_KI:
    case HR_KD:         (void)snprintf(b, TAM_ROT, "%s", fx(v, 2u));    break;
    case HR_MODO:       (void)snprintf(b, TAM_ROT, "%s", (v != 0u) ? "auto" : "manual"); break;
    case HR_PWM_MANUAL: (void)snprintf(b, TAM_ROT, "%s %%", fx(v, 1u)); break;
    default:            (void)snprintf(b, TAM_ROT, "%u", v);            break;
    }
    return b;
}

const char *fmt_ir(uint16_t dir, uint16_t v)
{
    char *b = siguiente();

    switch (dir) {
    case IR_SETPOINT:
    case IR_ALTURA:     (void)snprintf(b, TAM_ROT, "%s cm", fx(v, 1u));          break;
    case IR_ERROR:      (void)snprintf(b, TAM_ROT, "%s cm", fx((int16_t)v, 1u)); break;
    case IR_PWM:        (void)snprintf(b, TAM_ROT, "%s %%", fx(v, 1u));          break;
    case IR_ESTADO:     (void)snprintf(b, TAM_ROT, "0b%s", bin6(v));             break;
    default:            (void)snprintf(b, TAM_ROT, "%u", v);                     break;
    }
    return b;
}
