/* ============================================================================
 * hw_levitador.h  -  HC-SR04 (sin bloquear) y PWM del ventilador
 *
 * Componentes del TopDesign que usa (ver README):
 *   PWM_Fan  (24 MHz, Period 959 -> 25 kHz)  -> Pin_PWM -> IRLZ44N
 *   Timer_US (1 MHz, 16 bit, libre)            base de tiempo del eco
 *   Pin_TRIG (salida por software)
 *   Pin_ECHO (entrada, interrupcion en ambos flancos) -> isr_echo
 *
 * El ancho del eco se mide en la ISR con el Timer_US: no hay pulseIn() que
 * bloquee 12 ms, asi la PSoC sigue respondiendo Modbus mientras mide.
 * ==========================================================================*/
#ifndef HW_LEVITADOR_H
#define HW_LEVITADOR_H

#include <stdint.h>

void    hw_init(void);
void    hw_disparar(void);              /* pulso de 10 us en TRIG y arma la ISR */
uint8_t hw_eco(uint16_t *ancho_us);     /* 1 = eco completo, ancho en us        */
void    hw_eco_cancelar(void);          /* desarma (timeout sin eco)            */
void    hw_pwm_permil(uint16_t permil); /* 0..1000                              */

#endif /* HW_LEVITADOR_H */
