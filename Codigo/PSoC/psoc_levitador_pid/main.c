/* ============================================================================
 * main.c  -  Firmware del levitador neumatico en la PSoC 5LP (CY8CKIT-059)
 *            Esclavo Modbus RTU + control PID de altura
 *
 *  ESP32 --RS485--> MAX485 #2 --> UART_RS485 (19200 8N1) --> esclavo Modbus
 *  HC-SR04 --> hw_levitador (eco por interrupcion) --> control.c (PID 25 Hz)
 *  control.c --> PWM_Fan (25 kHz) --> IRLZ44N --> ventilador
 *  UART_DBG (115200, KitProg) --> log, consola y Serial Plotter
 *
 *  El bucle principal no bloquea (lo mas largo son los 10 us del TRIG):
 *    1) recepcion RS485 y respuesta Modbus (primero se responde, luego el log)
 *    2) consola
 *    3) control: disparo del HC-SR04 cada 40 ms; el paso del PID se ejecuta
 *       en cuanto llega el eco (o a los 15 ms si no llega)
 *
 *  Seguridad:
 *    - EMG = 1 o EN = 0 por Modbus: PWM a 0 en el mismo instante.
 *    - Sin tramas validas del maestro durante 1.5 s (habiendo tenido enlace):
 *      EN = 0 (timeout de seguridad, Guia v3 Seccion 0). Para reanudar el
 *      HMI vuelve a enviar EN:1.
 *    - Sensor sin eco ~0.8 s con el lazo cerrado: EN = 0 (control.c).
 * ==========================================================================*/
#include "project.h"
#include <stdint.h>
#include "modbus_slave.h"
#include "control.h"
#include "hw_levitador.h"
#include "planta_modelo.h"
#include "debug_log.h"
#include "consola.h"

/* ---------------- Configuracion ---------------- */
#define ETAPA_INICIAL     1u      /* 0 = sniffer, 1 = esclavo                */
#define LOG_INICIAL       1u      /* 0, 1 o 2                                */
#define FUENTE_INICIAL    FUENTE_REAL
#define USAR_LED          1u      /* 0 si no puso Pin_LED en el TopDesign    */

#define T35_MS            3u      /* fin de trama: >= 3.5 caracteres (2 ms)  */
#define ENLACE_TIMEOUT_MS 1500u   /* timeout de seguridad por Modbus         */
#define RESUMEN_MS        1000u
#define LED_MS            20u
#define CAL_ARRANQUE_MS   500u    /* espera antes de calibrar al arrancar    */

#define ECO_TIMEOUT_MS    15u     /* 12 ms (~2 m) + margen                   */
#define ECO_MAX_US        12000u  /* = ECHO_TIMEOUT_US del .ino              */
#define VEL_SONIDO_CM_US  0.0343f

void app_init(void);
void app_poll(void);

/* ---------------- Tiempo (SysTick a 1 ms) ---------------- */
static volatile uint32_t g_ms;

CY_ISR_PROTO(isr_systick);

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
static app_cfg_t cfg = { ETAPA_INICIAL, LOG_INICIAL, 0u, FUENTE_INICIAL, 0u, 0u, 0u };
static planta_t  planta_sim;

static uint8_t  rx_buf[MB_MAX_FRAME];
static uint16_t rx_len;
static uint8_t  rx_desborde;
static uint32_t t_ultimo_byte;

static uint8_t  tx_buf[MB_MAX_FRAME];

static uint16_t n_fc01, n_fc03, n_fc04;     /* lecturas en el ultimo segundo */
static uint32_t t_ultima_valida;
static uint8_t  enlace_caido = 1u;

static uint32_t t_ctrl, t_disparo, t_resumen;
static uint8_t  esperando_eco;
static uint16_t duty_permil;
static ctrl_modo_t modo_log = CM_CALIBRANDO;

static uint8_t  led_on;
static uint32_t t_led;

/* ---------------------------------------------------------------------------
 * Log de Modbus
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

/* Resumen cada segundo: trafico Modbus + estado del lazo */
static void resumen(void)
{
    const ctrl_info_t *ci = ctrl_info();
    uint16_t total = (uint16_t)(n_fc01 + n_fc03 + n_fc04);

    if ((cfg.etapa == 1u) && (cfg.log_nivel >= 1u) && (cfg.plot == 0u) &&
        ((total > 0u) || (ci->modo != CM_PARO))) {
        dbg_printf("[RD] FC04 x%u", n_fc04);
        if (n_fc03) { dbg_printf(" FC03 x%u", n_fc03); }
        if (n_fc01) { dbg_printf(" FC01 x%u", n_fc01); }
        dbg_printf(" ok=%u crc=%u | %s%s SP=%scm Alt=%scm PWM=%s%% Err=%scm Est=0b%s",
                   mb.tramas_ok, mb.errores_crc,
                   (cfg.fuente == FUENTE_SIM) ? "[SIM] " : "",
                   ctrl_nombre_modo(ci->modo),
                   fx(mb.ir[IR_SETPOINT], 1u), fx(mb.ir[IR_ALTURA], 1u),
                   fx(mb.ir[IR_PWM], 1u), fx((int16_t)mb.ir[IR_ERROR], 1u),
                   bin6(mb.ir[IR_ESTADO]));
        dbg_printf(" I=%s%% uFF=%s%%\r\n",
                   fx(ctrl_redondear(ci->I * 10.0f), 1u),
                   fx(ctrl_redondear(ci->uFF * 10.0f), 1u));
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

    /* 2) EMG o EN = 0: ventilador apagado ya, sin esperar el paso del PID */
    if ((mb.coil[COIL_EMG] != 0u) || (mb.coil[COIL_EN] == 0u)) {
        hw_pwm_permil(0u);
    }

    /* 3) Estadisticas */
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

    /* 4) Log, despues de haber respondido */
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
 * Control
 * -------------------------------------------------------------------------*/
static void log_control(void)
{
    const ctrl_info_t *ci = ctrl_info();
    uint16_t ev = ctrl_eventos();

    if (cfg.log_nivel == 0u) {
        modo_log = ci->modo;
        return;
    }
    if (ev & CTRL_EV_CAL_OK) {
        dbg_printf("[CTRL] Calibracion OK: base = %s cm | setpoint maximo = %s cm\r\n",
                   fx(ctrl_redondear(ci->base * 10.0f), 1u),
                   fx(ctrl_redondear(ci->spMax * 10.0f), 1u));
    }
    if (ev & CTRL_EV_CAL_FALLA) {
        dbg_printf("[CTRL] ERROR: el HC-SR04 no responde (revise TRIG/ECHO). Base = %s cm\r\n",
                   fx(ctrl_redondear(ci->base * 10.0f), 1u));
    }
    if (ev & CTRL_EV_SENSOR_PERDIDO) {
        dbg_puts("[CTRL] PARO DE SEGURIDAD: el sensor no responde (EN = 0)\r\n");
    }
    if (ev & CTRL_EV_FIN_RAMPA) {
        dbg_printf("[CTRL] Fin de rampa, el PID sostiene %s cm\r\n",
                   fx(ctrl_redondear(ci->sp * 10.0f), 1u));
    }
    if (ev & CTRL_EV_RAMPA_CANCEL) {
        dbg_puts("[CTRL] Rampa cancelada: llego un setpoint nuevo\r\n");
    }
    if (ev & CTRL_EV_FIN_BARRIDO) {
        dbg_puts("[CTRL] Fin del barrido (EN = 0)\r\n");
    }
    if (ev & CTRL_EV_UFF_ADOPTADO) {
        dbg_printf("[CTRL] Manual -> auto sin salto: uFF = %s %% (duty manual)\r\n",
                   fx(ctrl_redondear(ci->uFF * 10.0f), 1u));
    }
    if (ev & CTRL_EV_RST_INT) {
        dbg_puts("[CTRL] Integrador en cero\r\n");
    }
    if (ev & CTRL_EV_SP_RECORTADO) {
        dbg_printf("[CTRL] SP %s cm fuera del rango util -> el PID usa %s cm\r\n",
                   fx(mb.hr[HR_SETPOINT], 1u), fx(ctrl_redondear(ci->spMax * 10.0f), 1u));
    }
    if (ci->modo != modo_log) {
        dbg_printf("[CTRL] Modo %s -> %s\r\n", ctrl_nombre_modo(modo_log),
                   ctrl_nombre_modo(ci->modo));
        modo_log = ci->modo;
    }
}

static void plot(void)
{
    const ctrl_info_t *ci = ctrl_info();

    if (cfg.plot == 0u) {
        return;
    }
    dbg_printf("sp:%s,altura:%s,pwm:%s",
               fx(ctrl_redondear(ci->sp * 100.0f), 2u),
               fx(ctrl_redondear(ci->altura * 10.0f), 1u),
               fx(ctrl_redondear(ci->duty * 10.0f), 1u));
    if (cfg.plot_dist != 0u) {
        dbg_printf(",dist:%s", fx(ctrl_redondear(ci->dist * 10.0f), 1u));
    }
    dbg_puts("\r\n");
}

static void ejecutar_control(float dist_cm)
{
    duty_permil = ctrl_paso(&mb, dist_cm, enlace_caido);
    hw_pwm_permil((cfg.fuente == FUENTE_REAL) ? duty_permil : 0u);
    log_control();
    plot();
}

static void control_poll(uint32_t t)
{
    uint16_t us;
    float    d;

    if (cfg.fuente == FUENTE_SIM) {
        if ((uint32_t)(t - t_ctrl) >= CTRL_TS_MS) {
            t_ctrl += CTRL_TS_MS;
            ejecutar_control(planta_medir(&planta_sim));
            planta_avanzar(&planta_sim, (float)duty_permil / 10.0f, CTRL_TS_MS);
        }
        return;
    }

    if (esperando_eco == 0u) {
        if ((uint32_t)(t - t_ctrl) >= CTRL_TS_MS) {
            t_ctrl += CTRL_TS_MS;
            if ((uint32_t)(t - t_ctrl) >= CTRL_TS_MS) {
                t_ctrl = t;                       /* hubo un atraso grande */
            }
            t_disparo = t;
            hw_disparar();
            esperando_eco = 1u;
        }
    } else if (hw_eco(&us) != 0u) {
        esperando_eco = 0u;
        d = (us > ECO_MAX_US) ? -1.0f : ((float)us * VEL_SONIDO_CM_US / 2.0f);
        ejecutar_control(d);
    } else if ((uint32_t)(t - t_disparo) >= ECO_TIMEOUT_MS) {
        hw_eco_cancelar();
        esperando_eco = 0u;
        ejecutar_control(-1.0f);                  /* sin eco */
    }
}

static void cambiar_fuente(void)
{
    cfg.cambio_fuente = 0u;
    mb.coil[COIL_EN]  = 0u;
    hw_eco_cancelar();
    esperando_eco = 0u;
    hw_pwm_permil(0u);
    planta_init(&planta_sim, CTRL_BASE_DEFECTO_CM, ahora() + 1u);
    t_ctrl = ahora();
    ctrl_calibrar(CAL_ARRANQUE_MS);
}

/* ---------------------------------------------------------------------------
 * Arranque y bucle
 * -------------------------------------------------------------------------*/
void app_init(void)
{
    CyGlobalIntEnable;

    CyIntSetSysVector(CY_INT_SYSTICK_IRQN, isr_systick);
    (void)SysTick_Config(BCLK__BUS_CLK__HZ / 1000u);

    UART_DBG_Start();
    UART_RS485_Start();
    UART_RS485_ClearRxBuffer();
    hw_init();

    mb_init(&mb);
    ctrl_init();
    planta_init(&planta_sim, CTRL_BASE_DEFECTO_CM, 12345u);

    dbg_puts("\r\n==================================================\r\n"
             " PSoC 5LP - Levitador neumatico: esclavo Modbus + PID\r\n"
             " RS485: 19200 8N1, esclavo 1 | Mapa: guia v3\r\n"
             " PID: Ts 40 ms, ganancias en % de duty por cm (x100)\r\n"
             "==================================================\r\n");
    dbg_printf("etapa=%u log=%u fuente=%s | Kp=%s Ki=%s Kd=%s uFF=%s%%\r\n",
               cfg.etapa, cfg.log_nivel, (cfg.fuente == FUENTE_SIM) ? "sim" : "real",
               fx(mb.hr[HR_KP], 2u), fx(mb.hr[HR_KI], 2u), fx(mb.hr[HR_KD], 2u),
               fx(ctrl_redondear(ctrl_info()->uFF * 10.0f), 1u));
    consola_ayuda();
    dbg_puts("[CTRL] Calibrando fondo (pelota abajo, ventilador apagado)...\r\n");

    ctrl_calibrar(CAL_ARRANQUE_MS);
    t_ctrl    = ahora();
    t_resumen = ahora();
}

void app_poll(void)
{
    uint32_t t;

    rs485_poll();
    consola_poll(&cfg, &mb);
    if (cfg.cambio_fuente != 0u) {
        cambiar_fuente();
    }

    t = ahora();
    control_poll(t);

    if ((uint32_t)(t - t_resumen) >= RESUMEN_MS) {
        t_resumen += RESUMEN_MS;
        resumen();
    }

    if (!enlace_caido && ((uint32_t)(t - t_ultima_valida) > ENLACE_TIMEOUT_MS)) {
        enlace_caido = 1u;
        if (mb.coil[COIL_EN] != 0u) {
            mb.coil[COIL_EN] = 0u;                /* timeout de seguridad */
            hw_pwm_permil(0u);
            dbg_puts("[LINK] Sin tramas del maestro hace mas de 1.5 s -> "
                     "TIMEOUT DE SEGURIDAD: control apagado (EN = 0)\r\n");
        } else if ((cfg.etapa == 1u) && (cfg.log_nivel >= 1u)) {
            dbg_puts("[LINK] Sin tramas validas del maestro hace mas de 1.5 s\r\n");
        }
    }

#if USAR_LED
    if (led_on && ((uint32_t)(t - t_led) >= LED_MS)) {
        Pin_LED_Write(0u);
        led_on = 0u;
    }
#endif
}

#ifndef PRUEBA_HOST
int main(void)
{
    app_init();
    for (;;) {
        app_poll();
    }
}
#endif
