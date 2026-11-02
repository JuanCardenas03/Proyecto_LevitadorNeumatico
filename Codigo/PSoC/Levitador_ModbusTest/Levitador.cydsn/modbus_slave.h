/* ============================================================================
 * modbus_slave.h  -  Esclavo Modbus RTU (logica pura, sin hardware)
 * Levitador neumatico - firmware de prueba de comunicacion para PSoC 5LP
 *
 * Implementa el mapa de registros de la "Guia ESP32 + HMI v2" (Seccion 4):
 *   Holding 0..5, Input 0..6 (IR0 = copia del HR0), Coils 0..2
 * Funciones soportadas: 01, 03, 04, 05, 06 y 16.
 *
 * Este modulo NO toca la UART: recibe una trama completa y devuelve la
 * respuesta en un buffer. Asi el frente de control puede reutilizarlo tal
 * cual y solo cambiar la planta simulada por su PID real.
 * ==========================================================================*/
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>

#define MB_SLAVE_ID        1u
#define MB_NUM_HR          6u
#define MB_NUM_IR          7u
#define MB_NUM_COILS       3u
#define MB_MAX_FRAME       256u

/* Holding registers (FC 03 / 06 / 16) */
enum { HR_SETPOINT = 0, HR_KP, HR_KI, HR_KD, HR_MODO, HR_PWM_MANUAL };

/* Input registers (FC 04) - mismo orden que la linea DATA */
enum { IR_SETPOINT = 0, IR_ALTURA, IR_PWM, IR_ERROR, IR_ESTADO,
       IR_TRAMAS_OK, IR_ERR_CRC };

/* Coils (FC 01 / 05) */
enum { COIL_EN = 0, COIL_RST_INT, COIL_EMG };

/* Palabra de estado (IR 0x0004), Seccion 4.6 */
#define EST_SENSOR_OK      (1u << 0)
#define EST_EN_OBJETIVO    (1u << 1)
#define EST_PWM_SAT        (1u << 2)
#define EST_INT_SAT        (1u << 3)
#define EST_TIMEOUT_COM    (1u << 4)
#define EST_FUERA_RANGO    (1u << 5)

/* Codigos de excepcion Modbus */
#define MB_EXC_FUNCION     0x01u   /* funcion no soportada  */
#define MB_EXC_DIRECCION   0x02u   /* direccion invalida    */
#define MB_EXC_VALOR       0x03u   /* valor / longitud mala */

typedef struct {
    uint16_t hr[MB_NUM_HR];
    uint16_t ir[MB_NUM_IR];
    uint8_t  coil[MB_NUM_COILS];
    uint16_t tramas_ok;      /* tramas validas dirigidas a este esclavo */
    uint16_t errores_crc;    /* tramas con CRC malo o demasiado cortas  */
} mb_data_t;

typedef enum {
    MB_EV_OK = 0,        /* trama valida atendida                       */
    MB_EV_EXCEPCION,     /* trama valida, se respondio con excepcion    */
    MB_EV_CRC_MALO,      /* CRC no coincide: no se responde             */
    MB_EV_CORTA,         /* menos de 4 bytes: no se responde            */
    MB_EV_OTRO_ID        /* trama para otro esclavo: no se responde     */
} mb_ev_tipo_t;

/* Descripcion de lo que paso con la ultima trama (para el log) */
typedef struct {
    mb_ev_tipo_t tipo;
    uint8_t  id;
    uint8_t  fc;
    uint8_t  exc;
    uint8_t  broadcast;
    uint8_t  es_escritura;
    uint16_t dir;
    uint16_t cant;
    uint16_t valor;
    uint16_t crc_rx;
    uint16_t crc_calc;
} mb_event_t;

void     mb_init(mb_data_t *d);
void     mb_sync(mb_data_t *d);   /* IR0 = HR0, IR5/IR6 = contadores */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len);
uint16_t mb_hr_max(uint16_t dir);

/* Procesa una trama completa. Devuelve el numero de bytes a transmitir
 * (0 = no responder). 'resp' debe tener MB_MAX_FRAME bytes. */
uint16_t mb_process(mb_data_t *d, const uint8_t *req, uint16_t len,
                    uint8_t *resp, mb_event_t *ev);

#endif /* MODBUS_SLAVE_H */
