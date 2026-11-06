/* ============================================================================
 * planta_modelo.h  -  Levitador simulado (para probar sin ventilador)
 *
 * Modelo aerodinamico sencillo: ventilador de primer orden, velocidad del
 * aire que cae con la altura (fugas alrededor de la pelota), arrastre
 * cuadratico y gravedad. Con 51 % flota hacia la mitad del tubo, como el
 * uFF del .ino. Agrega ruido, ecos atipicos y lecturas perdidas del HC-SR04.
 *
 * NO es un modelo identificado del levitador real: sirve para probar la
 * logica (modos, Modbus, HMI, seguridad), no para sintonizar ganancias.
 * ==========================================================================*/
#ifndef PLANTA_MODELO_H
#define PLANTA_MODELO_H

#include <stdint.h>

typedef struct {
    float    h;         /* cm, altura de la pelota sobre el reposo */
    float    v;         /* cm/s                                    */
    float    w;         /* %, estado del ventilador (con inercia)  */
    float    base;      /* cm, distancia sensor->pelota en reposo  */
    uint32_t semilla;
    uint8_t  sin_eco;   /* 1 = sensor desconectado                 */
} planta_t;

void  planta_init(planta_t *p, float base_cm, uint32_t semilla);
void  planta_avanzar(planta_t *p, float duty_pct, uint16_t dt_ms);
float planta_medir(planta_t *p);   /* cm sensor->pelota, -1 = sin eco */

#endif /* PLANTA_MODELO_H */
