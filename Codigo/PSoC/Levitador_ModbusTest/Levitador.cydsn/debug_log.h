/* ============================================================================
 * debug_log.h  -  Salida legible por UART_DBG (KitProg USB-UART, 115200)
 * Nunca llamar estas funciones desde una interrupcion.
 * ==========================================================================*/
#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

void        dbg_puts(const char *s);
void        dbg_printf(const char *fmt, ...);
void        dbg_hex(const char *prefijo, const uint8_t *b, uint16_t n);

/* Formateadores (devuelven buffers rotativos: hasta 8 por linea) */
const char *fx(int32_t v, uint8_t decimales);        /* 226,1 -> "22.6" */
const char *bin6(uint16_t v);                        /* 3 -> "000011"   */
const char *fmt_hr(uint16_t dir, uint16_t v);        /* con unidades    */
const char *fmt_ir(uint16_t dir, uint16_t v);
const char *nombre_hr(uint16_t dir);
const char *nombre_ir(uint16_t dir);
const char *nombre_coil(uint16_t dir);
const char *nombre_exc(uint8_t exc);

#endif /* DEBUG_LOG_H */
