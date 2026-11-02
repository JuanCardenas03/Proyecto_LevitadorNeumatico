/* ============================================================================
 * plant_sim.h  -  Planta simulada (sustituye al PID real durante la prueba)
 *
 * Genera altura, PWM, error y palabra de estado coherentes con los holding y
 * coils recibidos, para que la grafica del HMI se mueva sin ventilador.
 * Desde la consola se pueden forzar valores (altura, error, estado).
 * ==========================================================================*/
#ifndef PLANT_SIM_H
#define PLANT_SIM_H

#include <stdint.h>
#include "modbus_slave.h"

typedef struct {
    uint8_t  alt_activa;   int16_t  alt_valor;   /* altura fija (cm x 10)   */
    uint8_t  err_activo;   int16_t  err_valor;   /* error forzado (cm x 10) */
    uint8_t  est_activo;   uint16_t est_valor;   /* estado forzado (bitmap) */
} plant_override_t;

void              plant_init(void);
void              plant_step(mb_data_t *d, uint16_t dt_ms, uint8_t enlace_caido);
plant_override_t *plant_overrides(void);
void              plant_clear_overrides(void);
uint16_t          plant_resets_integrador(void);  /* pulsos de RST recibidos */

#endif /* PLANT_SIM_H */
