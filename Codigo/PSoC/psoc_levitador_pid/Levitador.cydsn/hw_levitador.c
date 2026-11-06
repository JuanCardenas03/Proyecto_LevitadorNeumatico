/* ============================================================================
 * hw_levitador.c  -  HC-SR04 (sin bloquear) y PWM del ventilador
 * ==========================================================================*/
#include "project.h"
#include "hw_levitador.h"

#define PWM_CUENTAS  960u   /* Period = 959 en el TopDesign: 24 MHz / 960 = 25 kHz */

/* 0 = inactivo, 1 = armado (espera flanco de subida), 2 = midiendo */
static volatile uint8_t  eco_estado;
static volatile uint8_t  eco_listo;
static volatile uint16_t eco_t_sube;
static volatile uint16_t eco_ancho;

CY_ISR_PROTO(isr_eco_handler);

/* Flanco en ECHO. Timer_US cuenta hacia ABAJO a 1 MHz y da la vuelta cada
 * 65.536 ms: el ancho es (t_subida - t_bajada) en aritmetica de 16 bits. */
CY_ISR(isr_eco_handler)
{
    uint16_t t = (uint16_t)Timer_US_ReadCounter();

    (void)Pin_ECHO_ClearInterrupt();

    if (Pin_ECHO_Read() != 0u) {
        if (eco_estado == 1u) {
            eco_t_sube = t;
            eco_estado = 2u;
        }
    } else if (eco_estado == 2u) {
        eco_ancho  = (uint16_t)(eco_t_sube - t);
        eco_listo  = 1u;
        eco_estado = 0u;
    }
}

void hw_init(void)
{
    PWM_Fan_Start();
    hw_pwm_permil(0u);

    Timer_US_Start();

    Pin_TRIG_Write(0u);
    (void)Pin_ECHO_ClearInterrupt();
    isr_echo_ClearPending();
    isr_echo_StartEx(isr_eco_handler);
}

void hw_disparar(void)
{
    isr_echo_Disable();
    eco_listo  = 0u;
    eco_estado = 1u;
    isr_echo_Enable();

    Pin_TRIG_Write(1u);
    CyDelayUs(10u);
    Pin_TRIG_Write(0u);
}

uint8_t hw_eco(uint16_t *ancho_us)
{
    uint8_t listo = 0u;

    isr_echo_Disable();
    if (eco_listo != 0u) {
        *ancho_us = eco_ancho;
        eco_listo = 0u;
        listo     = 1u;
    }
    isr_echo_Enable();
    return listo;
}

void hw_eco_cancelar(void)
{
    isr_echo_Disable();
    eco_estado = 0u;
    eco_listo  = 0u;
    isr_echo_Enable();
}

void hw_pwm_permil(uint16_t permil)
{
    if (permil > 1000u) {
        permil = 1000u;
    }
    /* Compare mode "Less": salida en alto mientras contador < compare.
     * compare = 960 -> 100 %; compare = 0 -> 0 %. */
    PWM_Fan_WriteCompare((uint16)(((uint32_t)permil * PWM_CUENTAS + 500u) / 1000u));
}
