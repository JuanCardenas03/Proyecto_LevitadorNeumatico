/* ============================================================================
 * consola.h  -  Comandos por el puerto de depuracion (KitProg USB-UART)
 * ==========================================================================*/
#ifndef CONSOLA_H
#define CONSOLA_H

#include <stdint.h>
#include "modbus_slave.h"

#define FUENTE_REAL  0u   /* HC-SR04 + ventilador                     */
#define FUENTE_SIM   1u   /* planta simulada (ventilador apagado)     */

typedef struct {
    uint8_t etapa;          /* 0 = sniffer pasivo, 1 = esclavo Modbus          */
    uint8_t log_nivel;      /* 0 = silencio, 1 = eventos + resumen, 2 = todo   */
    uint8_t mute;           /* 1 = no responder (simula enlace caido)          */
    uint8_t fuente;         /* FUENTE_REAL o FUENTE_SIM                        */
    uint8_t plot;           /* 1 = linea sp,altura,pwm cada 40 ms              */
    uint8_t plot_dist;      /* 1 = agrega la distancia filtrada al plot        */
    uint8_t cambio_fuente;  /* lo pone la consola, lo atiende main             */
} app_cfg_t;

void consola_ayuda(void);
void consola_dump(const app_cfg_t *cfg, const mb_data_t *d);
void consola_poll(app_cfg_t *cfg, mb_data_t *d);

#endif /* CONSOLA_H */
