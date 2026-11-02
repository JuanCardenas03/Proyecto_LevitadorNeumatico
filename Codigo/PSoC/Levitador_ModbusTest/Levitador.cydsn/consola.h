/* ============================================================================
 * consola.h  -  Comandos por el puerto de depuracion (KitProg USB-UART)
 * ==========================================================================*/
#ifndef CONSOLA_H
#define CONSOLA_H

#include <stdint.h>
#include "modbus_slave.h"

typedef struct {
    uint8_t etapa;      /* 0 = sniffer pasivo, 1 = esclavo Modbus          */
    uint8_t log_nivel;  /* 0 = silencio, 1 = escrituras + resumen, 2 = todo */
    uint8_t mute;       /* 1 = no responder (simula enlace caido)           */
} app_cfg_t;

void consola_ayuda(void);
void consola_dump(const app_cfg_t *cfg, const mb_data_t *d);
void consola_poll(app_cfg_t *cfg, mb_data_t *d);

#endif /* CONSOLA_H */
