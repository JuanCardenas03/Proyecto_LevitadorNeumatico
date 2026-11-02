/* ============================================================================
 * main.c  -  Firmware de PRUEBA de comunicacion Modbus RTU para la PSoC 5LP
 *            (CY8CKIT-059) - Levitador neumatico
 *
 *  ESP32 --RS485--> MAX485 #2 --> UART_RS485 (19200 8N1) --> esclavo Modbus
 *                                         |
 *                                         +--> UART_DBG (115200) --> KitProg
 *
 *  Etapa 0: sniffer pasivo. Muestra cada trama en hex con su CRC. No responde.
 *  Etapa 1: esclavo Modbus con el mapa v2 + planta simulada.
 *  Se cambia en caliente con el comando "etapa 0|1" en el monitor serie.
 *
 *  Regla: primero se RESPONDE por RS485 y despues se escribe el log.
 *  Ningun printf dentro de interrupciones.
 *
 *  Componentes requeridos en TopDesign (ver README):
 *    UART_RS485, UART_DBG, Pin_LED (P2[1], LED de la placa)
 * ==========================================================================*/
#include "project.h"
#include <stdint.h>
#include "modbus_slave.h"
#include "plant_sim.h"
#include "debug_log.h"
#include "consola.h"

/* ---------------- Configuracion ---------------- */
#define ETAPA_INICIAL     1u      /* 0 = sniffer, 1 = esclavo                */
#define LOG_INICIAL       1u      /* 0, 1 o 2                                */
#define USAR_LED          1u      /* 0 si no puso Pin_LED en el TopDesign    */

#define T35_MS            3u      /* fin de trama: >= 3.5 caracteres. A 19200
                                     son 2.0 ms; con tick de 1 ms, ">= 3"
                                     garantiza entre 2 y 3 ms de silencio    */
#define ENLACE_TIMEOUT_MS 1000u   /* sin tramas validas -> bit 4 del estado  */
#define RESUMEN_MS        1000u
#define PLANTA_MS         10u
#define LED_MS            20u

/* ---------------- Tiempo (SysTick a 1 ms) ---------------- */
static volatile uint32_t g_ms;

CY_ISR(isr_systick)
{
    g_ms++;
}

static uint32_t ahora(void)
{
    return g_ms;
}

/* ---------------- Estado ---------------- */
static mb_data_t mb;
static app_cfg_t cfg = { ETAPA_INICIAL, LOG_INICIAL, 0u };

static uint8_t  rx_buf[MB_MAX_FRAME];
static uint16_t rx_len;
static uint8_t  rx_desborde;
static uint32_t t_ultimo_byte;

static uint8_t  tx_buf[MB_MAX_FRAME];

static uint16_t n_fc01, n_fc03, n_fc04;     /* lecturas en el ultimo segundo */
static uint32_t t_ultima_valida;
static uint8_t  enlace_caido = 1u;

static uint8_t  led_on;
static uint32_t t_led;

/* ---------------------------------------------------------------------------
 * Log
 * -------------------------------------------------------------------------*/
static void log_sniffer(void)
{
    uint16_t crc_rx = 0u, crc_calc = 0u;
    uint8_t  crc_ok = 0u;

    if (cfg.log_nivel == 0u) {
        return;
    }
    if (rx_len >= 4u) {
        crc_rx   = (uint16_t)(rx_buf[rx_len - 2u] | ((uint16_t)rx_buf[rx_len - 1u] << 8));
        crc_calc = mb_crc16(rx_buf, (uint16_t)(rx_len - 2u));
        crc_ok   = (crc_rx == crc_calc) ? 1u : 0u;
    }

    if (crc_ok && (rx_len >= 6u)) {
        dbg_printf("[SNIF] %u B%s | CRC OK | id=%u FC%02X dir=0x%04X dato=%u\r\n",
                   rx_len, rx_desborde ? " (desborde)" : "",
                   rx_buf[0], rx_buf[1],
                   (unsigned)((rx_buf[2] << 8) | rx_buf[3]),
                   (unsigned)((rx_buf[4] << 8) | rx_buf[5]));
    } else if (rx_len >= 4u) {
        dbg_printf("[SNIF] %u B%s | CRC MALO (rx=0x%04X calc=0x%04X)\r\n",
                   rx_len, rx_desborde ? " (desborde)" : "", crc_rx, crc_calc);
    } else {
        dbg_printf("[SNIF] %u B | trama demasiado corta (ruido?)\r\n", rx_len);
    }
    dbg_hex("       ", rx_buf, rx_len);
}

static void log_escritura(const mb_event_t *ev, const char *sufijo)
{
    uint16_t i;

    switch (ev->fc) {
    case 0x05u:
        dbg_printf("[RX] FC05 Coil%u %s = %s %s\r\n", ev->dir, nombre_coil(ev->dir),
                   (ev->valor == 0xFF00u) ? "ON" : "OFF", sufijo);
        break;
    case 0x06u:
        dbg_printf("[RX] FC06 HR%u %s = %s [crudo=%u] %s\r\n", ev->dir,
                   nombre_hr(ev->dir), fmt_hr(ev->dir, ev->valor), ev->valor, sufijo);
        break;
    case 0x10u:
        dbg_printf("[RX] FC16 HR%u..HR%u %s\r\n", ev->dir,
                   (unsigned)(ev->dir + ev->cant - 1u), sufijo);
        for (i = ev->dir; i < (uint16_t)(ev->dir + ev->cant); i++) {
            dbg_printf("       HR%u %s = %s [crudo=%u]\r\n", i, nombre_hr(i),
                       fmt_hr(i, mb.hr[i]), mb.hr[i]);
        }
        break;
    default:
        break;
    }
}

static void log_evento(const mb_event_t *ev, uint16_t n_tx, uint8_t respondio)
{
    const char *sufijo;

    if (cfg.log_nivel == 0u) {
        return;
    }
    if (cfg.log_nivel >= 2u) {
        dbg_hex("[RX hex] ", rx_buf, rx_len);
        if (respondio) {
            dbg_hex("[TX hex] ", tx_buf, n_tx);
        }
    }

    if (respondio)            { sufijo = "-> respondido OK"; }
    else if (ev->broadcast)   { sufijo = "-> broadcast, sin respuesta"; }
    else if (cfg.mute)        { sufijo = "-> MUTE, sin respuesta"; }
    else                      { sufijo = ""; }

    switch (ev->tipo) {
    case MB_EV_CORTA:
        dbg_printf("[ERR] Trama demasiado corta (%u B): ", rx_len);
        dbg_hex("", rx_buf, rx_len);
        break;
    case MB_EV_CRC_MALO:
        dbg_printf("[ERR] CRC invalido rx=0x%04X calc=0x%04X (%u B): ",
                   ev->crc_rx, ev->crc_calc, rx_len);
        dbg_hex("", rx_buf, rx_len);
        break;
    case MB_EV_OTRO_ID:
        if (cfg.log_nivel >= 2u) {
            dbg_printf("[--] Trama para el esclavo %u (ignorada)\r\n", ev->id);
        }
        break;
    case MB_EV_EXCEPCION:
        dbg_printf("[EXC] FC%02X dir=%u cant=%u valor=%u -> excepcion 0x%02X (%s) %s\r\n",
                   ev->fc, ev->dir, ev->cant, ev->valor, ev->exc,
                   nombre_exc(ev->exc), sufijo);
        break;
    case MB_EV_OK:
    default:
        if (ev->es_escritura) {
            log_escritura(ev, sufijo);
        } else if (cfg.log_nivel >= 2u) {
            dbg_printf("[RD] FC%02X dir=%u cant=%u %s (%u B)\r\n",
                       ev->fc, ev->dir, ev->cant, sufijo, n_tx);
        }
        break;
    }
}

static void resumen(void)
{
    uint16_t total = (uint16_t)(n_fc01 + n_fc03 + n_fc04);

    if ((cfg.etapa == 1u) && (cfg.log_nivel >= 1u) && (total > 0u)) {
        dbg_printf("[RD] 1 s: FC04 x%u", n_fc04);
        if (n_fc03) { dbg_printf(" FC03 x%u", n_fc03); }
        if (n_fc01) { dbg_printf(" FC01 x%u", n_fc01); }
        dbg_printf(" | ok=%u crc=%u | SP=%scm Alt=%scm PWM=%s%% Err=%scm Est=0b%s\r\n",
                   mb.tramas_ok, mb.errores_crc,
                   fx(mb.ir[IR_SETPOINT], 1u), fx(mb.ir[IR_ALTURA], 1u),
                   fx(mb.ir[IR_PWM], 1u), fx((int16_t)mb.ir[IR_ERROR], 1u),
                   bin6(mb.ir[IR_ESTADO]));
    }
    n_fc01 = 0u;
    n_fc03 = 0u;
    n_fc04 = 0u;
}

/* ---------------------------------------------------------------------------
 * Recepcion RS485 y atencion de tramas
 * -------------------------------------------------------------------------*/
static void procesar_trama(void)
{
    mb_event_t ev;
    uint16_t   n;
    uint8_t    respondio = 0u;

    if (cfg.etapa == 0u) {
        log_sniffer();
        return;
    }

    n = mb_process(&mb, rx_buf, rx_len, tx_buf, &ev);

    /* 1) RESPONDER primero (el tx_en del UART maneja DE/RE del MAX485) */
    if ((n > 0u) && (cfg.mute == 0u)) {
        UART_RS485_PutArray(tx_buf, (uint8)n);
        respondio = 1u;
#if USAR_LED
        Pin_LED_Write(1u);
        led_on = 1u;
        t_led  = ahora();
#endif
    }

    /* 2) Estadisticas */
    if ((ev.tipo == MB_EV_OK) || (ev.tipo == MB_EV_EXCEPCION)) {
        t_ultima_valida = ahora();
        if (enlace_caido) {
            enlace_caido = 0u;
            if (cfg.log_nivel >= 1u) {
                dbg_puts("[LINK] Recibiendo tramas validas del maestro\r\n");
            }
        }
        if ((ev.tipo == MB_EV_OK) && !ev.es_escritura) {
            if (ev.fc == 0x01u) { n_fc01++; }
            if (ev.fc == 0x03u) { n_fc03++; }
            if (ev.fc == 0x04u) { n_fc04++; }
        }
    }

    /* 3) Log, despues de haber respondido */
    log_evento(&ev, n, respondio);
}

static void rs485_poll(void)
{
    uint8_t b;

    while (UART_RS485_GetRxBufferSize() != 0u) {
        b = UART_RS485_ReadRxData();
        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = b;
        } else {
            rx_desborde = 1u;
        }
        t_ultimo_byte = ahora();
    }

    if ((rx_len > 0u) && ((uint32_t)(ahora() - t_ultimo_byte) >= T35_MS)) {
        procesar_trama();
        rx_len      = 0u;
        rx_desborde = 0u;
    }
}

/* ---------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------*/
int main(void)
{
    uint32_t t_planta, t_resumen, t;

    CyGlobalIntEnable;

    CyIntSetSysVector(CY_INT_SYSTICK_IRQN, isr_systick);
    (void)SysTick_Config(BCLK__BUS_CLK__HZ / 1000u);

    UART_DBG_Start();
    UART_RS485_Start();
    UART_RS485_ClearRxBuffer();

    mb_init(&mb);
    plant_init();

    dbg_puts("\r\n==================================================\r\n"
             " PSoC 5LP - Prueba esclavo Modbus RTU (levitador)\r\n"
             " RS485: 19200 8N1, esclavo 1 | Mapa: guia v2\r\n"
             "==================================================\r\n");
    dbg_printf("etapa=%u (%s) log=%u\r\n", cfg.etapa,
               cfg.etapa ? "esclavo Modbus" : "sniffer pasivo", cfg.log_nivel);
    consola_ayuda();

    t_planta  = ahora();
    t_resumen = ahora();

    for (;;) {
        rs485_poll();
        consola_poll(&cfg, &mb);

        t = ahora();

        if ((uint32_t)(t - t_planta) >= PLANTA_MS) {
            t_planta += PLANTA_MS;
            plant_step(&mb, PLANTA_MS, enlace_caido);
        }

        if ((uint32_t)(t - t_resumen) >= RESUMEN_MS) {
            t_resumen += RESUMEN_MS;
            resumen();
        }

        if (!enlace_caido && ((uint32_t)(t - t_ultima_valida) > ENLACE_TIMEOUT_MS)) {
            enlace_caido = 1u;
            if ((cfg.etapa == 1u) && (cfg.log_nivel >= 1u)) {
                dbg_puts("[LINK] Sin tramas validas del maestro hace mas de 1 s\r\n");
            }
        }

#if USAR_LED
        if (led_on && ((uint32_t)(t - t_led) >= LED_MS)) {
            Pin_LED_Write(0u);
            led_on = 0u;
        }
#endif
    }
}
