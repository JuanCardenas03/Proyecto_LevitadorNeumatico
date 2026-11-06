/* Prueba 2: firmware COMPLETO de la PSoC (main.c, modbus_slave.c, control.c,
 * hw_levitador.c, consola.c, debug_log.c, planta_modelo.c sin cambios)
 * corriendo en el PC sobre APIs falsas de PSoC Creator.
 *
 *  - Reloj simulado con resolucion de 100 us; SysTick cada 1 ms.
 *  - HC-SR04 emulado a nivel de pines: TRIG -> flancos de ECHO -> isr_echo,
 *    con el Timer_US de 1 MHz (cuenta hacia abajo, 16 bits).
 *  - Ventilador + pelota: planta_modelo movida por el compare del PWM.
 *  - ESP32 emulada: FC04 de 7 registros cada 500 ms y escrituras FC05/FC06
 *    como las manda el firmware v1.1 (RST = ON, 50 ms, OFF).
 *  - Consola: se escriben comandos por el UART_DBG.
 */
#include "project.h"
#include "modbus_slave.h"
#include "control.h"
#include "planta_modelo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void app_init(void);
void app_poll(void);

/* ======================= estado de la simulacion ======================= */
static uint64_t sim_us, lectura_us;
static cyisraddress systick_fn, eco_fn;

static uint8_t  rs_rx[1024]; static int rs_h, rs_t;
static uint8_t  rs_tx[256];  static int rs_tx_len; static uint64_t rs_tx_us; static int rs_tx_nuevo;

static char     con_q[512]; static int con_h, con_t;
static char    *logbuf; static size_t loglen; static FILE *flog;

static int      trig, echo, pend_sube, pend_baja;
static uint64_t t_sube, t_baja;
static uint16_t pwm_cmp;
static planta_t real;
static int      sensor_desconectado;

/* ============================ stubs PSoC ============================ */
cyisraddress CyIntSetSysVector(uint8 n, cyisraddress f) { (void)n; systick_fn = f; return 0; }
uint32 SysTick_Config(uint32 t) { (void)t; return 0; }
void CyDelayUs(uint16 us) { (void)us; }

void  UART_RS485_Start(void) {}
void  UART_RS485_ClearRxBuffer(void) { rs_h = rs_t = 0; }
uint8 UART_RS485_GetRxBufferSize(void) { return (uint8)(rs_t - rs_h); }
uint8 UART_RS485_ReadRxData(void) { return rs_rx[rs_h++]; }
void  UART_RS485_PutArray(const uint8 *s, uint8 n) { memcpy(rs_tx, s, n); rs_tx_len = n; rs_tx_us = sim_us; rs_tx_nuevo = 1; }

void  UART_DBG_Start(void) {}
void  UART_DBG_PutString(const char *s) {
    size_t n = strlen(s);
    logbuf = realloc(logbuf, loglen + n + 1); memcpy(logbuf + loglen, s, n + 1); loglen += n;
    fputs(s, flog);
}
void  UART_DBG_PutChar(char c) { char s[2] = { c, 0 }; UART_DBG_PutString(s); }
uint8 UART_DBG_GetRxBufferSize(void) { return (uint8)(con_t - con_h); }
uint8 UART_DBG_ReadRxData(void) { return (uint8)con_q[con_h++]; }

void  Pin_LED_Write(uint8 v) { (void)v; }
static void programar_eco(void) {
    float d;
    if (sensor_desconectado) { pend_sube = pend_baja = 0; return; }
    d = planta_medir(&real);
    t_sube = sim_us + 460;                                   /* rafaga de 8 ciclos */
    t_baja = t_sube + ((d < 0) ? 38000u : (uint64_t)(d / 0.01715f));  /* sin eco: 38 ms */
    pend_sube = pend_baja = 1;
}
void  Pin_TRIG_Write(uint8 v) { if (trig && !v) programar_eco(); trig = v; }
uint8 Pin_ECHO_Read(void) { return (uint8)echo; }
uint8 Pin_ECHO_ClearInterrupt(void) { return 0; }

void   PWM_Fan_Start(void) {}
void   PWM_Fan_WriteCompare(uint16 c) { pwm_cmp = c; }
void   Timer_US_Start(void) {}
uint16 Timer_US_ReadCounter(void) { return (uint16)(0xFFFFu - (uint16)(lectura_us & 0xFFFFu)); }
void   isr_echo_StartEx(cyisraddress f) { eco_fn = f; }
void   isr_echo_ClearPending(void) {}
void   isr_echo_Enable(void) {}
void   isr_echo_Disable(void) {}

/* ============================ utilidades ============================ */
static int n_pasa, n_falla;
static void check(const char *nombre, int ok, const char *detalle) {
    printf("  [%s] %-58s %s\n", ok ? "PASA " : "FALLA", nombre, detalle ? detalle : "");
    if (ok) n_pasa++; else n_falla++;
}
static int log_tiene(const char *s, size_t desde) { return logbuf && strstr(logbuf + desde, s) != NULL; }

static uint16_t crc16(const uint8_t *b, int n) {
    uint16_t c = 0xFFFF; int i, k;
    for (i = 0; i < n; i++) { c ^= b[i]; for (k = 0; k < 8; k++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1); }
    return c;
}

/* ---- ESP32 emulada ---- */
static struct { int activo; uint8_t fc; uint64_t t_us; int espera_resp; } pend;
static int esp_activo = 1;
static uint16_t ir_vistos[7]; static int n_polls, n_resp_ok, n_timeouts, n_crc_resp_malo;
static uint64_t lat_max_us; static int ult_exc; static uint8_t coils_vistos; static uint16_t hr_visto;
static int16_t err_min_visto = 32767;

static void enviar(uint8_t *f, int n, int espera_resp) {
    uint16_t c = crc16(f, n);
    if (rs_h == rs_t) rs_h = rs_t = 0;
    f[n] = (uint8_t)(c & 0xFF); f[n + 1] = (uint8_t)(c >> 8);
    memcpy(&rs_rx[rs_t], f, (size_t)(n + 2)); rs_t += n + 2;
    pend.activo = 1; pend.fc = f[1]; pend.t_us = sim_us; pend.espera_resp = espera_resp;
    ult_exc = 0;
}
static void fc06(uint16_t dir, uint16_t v) { uint8_t f[10] = {1, 6, (uint8_t)(dir >> 8), (uint8_t)dir, (uint8_t)(v >> 8), (uint8_t)v}; enviar(f, 6, 1); }
static void fc05(uint16_t dir, int on) { uint8_t f[10] = {1, 5, 0, (uint8_t)dir, on ? 0xFF : 0, 0}; enviar(f, 6, 1); }
static void fc04(uint16_t ini, uint16_t n) { uint8_t f[10] = {1, 4, 0, (uint8_t)ini, 0, (uint8_t)n}; enviar(f, 6, 1); }
static void fc03(uint16_t ini, uint16_t n) { uint8_t f[10] = {1, 3, 0, (uint8_t)ini, 0, (uint8_t)n}; enviar(f, 6, 1); }
static void fc01(uint16_t ini, uint16_t n) { uint8_t f[10] = {1, 1, 0, (uint8_t)ini, 0, (uint8_t)n}; enviar(f, 6, 1); }

static void revisar_respuesta(void) {
    if (pend.activo && rs_tx_nuevo) {
        uint64_t lat = rs_tx_us - pend.t_us;
        rs_tx_nuevo = 0; pend.activo = 0;
        if (lat > lat_max_us) lat_max_us = lat;
        if (crc16(rs_tx, rs_tx_len - 2) != (uint16_t)(rs_tx[rs_tx_len - 2] | (rs_tx[rs_tx_len - 1] << 8))) { n_crc_resp_malo++; return; }
        if (rs_tx[1] & 0x80) { ult_exc = rs_tx[2]; return; }
        n_resp_ok++;
        if (rs_tx[1] == 4 && rs_tx[2] == 14) {
            int i; for (i = 0; i < 7; i++) ir_vistos[i] = (uint16_t)((rs_tx[3 + 2 * i] << 8) | rs_tx[4 + 2 * i]);
            if ((int16_t)ir_vistos[3] < err_min_visto) err_min_visto = (int16_t)ir_vistos[3];
        }
        if (rs_tx[1] == 1) coils_vistos = rs_tx[3];
        if (rs_tx[1] == 3) hr_visto = (uint16_t)((rs_tx[3] << 8) | rs_tx[4]);
    } else if (pend.activo && (sim_us - pend.t_us) > 100000u) {
        if (pend.espera_resp) n_timeouts++;
        pend.activo = 0;
    }
}

static void consola(const char *s) {
    size_t n = strlen(s);
    if (con_h == con_t) con_h = con_t = 0;
    memcpy(&con_q[con_t], s, n); con_t += (int)n; con_q[con_t++] = '\n';
}

/* ---- registro para la grafica ---- */
static FILE *fcsv;
/* metricas de escalon */
typedef struct { double t0, sp, h0, max_dev, overshoot, t_estab; int activo; } escalon_t;
static escalon_t esc;
static void iniciar_escalon(double t, double sp) { esc.t0 = t; esc.sp = sp; esc.h0 = real.h; esc.overshoot = 0; esc.t_estab = -1; esc.activo = 1; }
static double t_fuera_banda;

static void muestrear(uint32_t ms) {
    const ctrl_info_t *ci = ctrl_info();
    if (ms % 20 == 0)
        fprintf(fcsv, "%.3f,%.2f,%.2f,%.2f,%.1f,%d\n", ms / 1000.0, ci->sp, ci->altura, real.h, ci->duty, (int)ci->modo);
    if (esc.activo) {
        double dir = (esc.sp >= esc.h0) ? 1 : -1;
        double sobre = (real.h - esc.sp) * dir;
        if (sobre > esc.overshoot) esc.overshoot = sobre;
        if (fabs(real.h - esc.sp) > 0.5) t_fuera_banda = ms / 1000.0;
    }
}
static void cerrar_escalon(const char *nombre) {
    char det[128];
    esc.t_estab = t_fuera_banda - esc.t0;
    snprintf(det, sizeof det, "sobrepico %.1f cm, establecimiento (+/-5 mm) %.1f s", esc.overshoot, esc.t_estab);
    printf("  [INFO ] %-58s %s\n", nombre, det);   /* depende del modelo, no del firmware */
    esc.activo = 0;
}

/* banda de error en una ventana */
static double max_dev_ventana(double sp, double *acc, int *n, int reset) {
    if (reset) { *acc = 0; *n = 0; return 0; }
    double d = fabs(real.h - sp); if (d > *acc) *acc = d; (*n)++; return *acc;
}

/* ============================= escenario ============================= */
static size_t marca;
static double dev; static int ndev; static int midiendo_banda; static double sp_banda;
static uint16_t pwm_max_sim; static int en_sim;

static void escenario(uint32_t ms) {
    char det[160];
    const ctrl_info_t *ci = ctrl_info();

    /* sondeo de la ESP32 cada 500 ms */
    if (esp_activo && (ms % 500) == 250 && !pend.activo) { fc04(0, 7); n_polls++; }

    if (midiendo_banda) max_dev_ventana(sp_banda, &dev, &ndev, 0);
    if (en_sim && pwm_cmp > pwm_max_sim) pwm_max_sim = pwm_cmp;

    switch (ms) {
    case 1600:
        snprintf(det, sizeof det, "base medida %.2f cm (real 36.40), spMax %.1f", ci->base, ci->spMax);
        check("Calibracion del fondo al arrancar (no bloqueante)", fabs(ci->base - 36.4) < 0.3 && log_tiene("Calibracion OK", 0), det);
        check("Respondio Modbus durante la calibracion", n_timeouts == 0 && n_resp_ok >= 2, NULL);
        break;
    case 2000: fc06(HR_KP, 100); break;
    case 2020: fc06(HR_KI, 60); break;
    case 2040: fc06(HR_KD, 20); break;
    case 2060: fc06(HR_MODO, 1); break;
    case 2080: fc06(HR_SETPOINT, 150); break;
    case 2100: fc05(COIL_EN, 1); iniciar_escalon(2.1, 15.0); break;
    case 2260:
        check("IR0 = HR0 tras escribir SP:150", ir_vistos[0] == 150, NULL); break;
    case 9000:
        cerrar_escalon("Escalon 0 -> 15 cm");
        midiendo_banda = 1; sp_banda = 15; max_dev_ventana(0, &dev, &ndev, 1); break;
    case 9050:
        midiendo_banda = 0;
        snprintf(det, sizeof det, "IR4 = 0x%02X", ir_vistos[4]);
        check("Estado: sensor OK + pelota en objetivo (bits 0 y 1)", (ir_vistos[4] & 0x03) == 0x03, det);
        fc06(HR_SETPOINT, 250); iniciar_escalon(9.05, 25.0); break;
    case 16000: cerrar_escalon("Escalon 15 -> 25 cm"); break;
    case 16050: fc06(HR_SETPOINT, 80); iniciar_escalon(16.05, 8.0); break;
    case 22000:
        cerrar_escalon("Escalon 25 -> 8 cm");
        snprintf(det, sizeof det, "minimo visto %d (= %.1f cm)", err_min_visto, err_min_visto / 10.0);
        check("IR3 (error) llega negativo en complemento a 2", err_min_visto < 0, det);
        break;
    case 22100: fc06(HR_MODO, 0); break;
    case 22120: fc06(HR_PWM_MANUAL, 520); break;
    case 22300:
        snprintf(det, sizeof det, "duty %.1f %%, compare %u", ci->duty, pwm_cmp);
        check("Modo manual: PWM = HR5 (52.0 %)", fabs(ci->duty - 52.0) < 0.01 && pwm_cmp == 499, det); break;
    case 25900: fc06(HR_SETPOINT, ir_vistos[1]); break;   /* el HMI fija sp = altura actual */
    case 26000: fc06(HR_MODO, 1); marca = loglen; break;
    case 26050:
        snprintf(det, sizeof det, "uFF %.1f %%, primer duty PID %.1f %%", ci->uFF, ci->duty);
        check("Manual -> auto sin salto (adopta el duty como uFF)", fabs(ci->uFF - 52.0) < 0.01 && fabs(ci->duty - 52.0) < 3.0 && ci->modo == CM_PID, det); break;
    case 30000: fc05(COIL_RST_INT, 1); marca = loglen; break;
    case 30050: fc05(COIL_RST_INT, 0); break;
    case 30100:
        check("Pulso RST reinicia el integrador", ci->pulsos_rst == 1 && log_tiene("Integrador en cero", marca), NULL); break;
    case 32000: fc05(COIL_EMG, 1); break;
    case 32005:
        snprintf(det, sizeof det, "compare %u a los 5 ms", pwm_cmp);
        check("EMG:1 apaga el PWM de inmediato (sin esperar al PID)", pwm_cmp == 0, det); break;
    case 32500: check("Con EMG el lazo queda en PARO", ci->modo == CM_PARO && pwm_cmp == 0, NULL); break;
    case 33000: fc05(COIL_EMG, 0); break;
    case 33100: check("Al soltar EMG (EN sigue en 1) el PID retoma", ci->modo == CM_PID, NULL); break;
    case 36000: esp_activo = 0; marca = loglen; break;            /* ESP32 desconectada */
    case 37000: check("A ~1.25 s sin tramas el control sigue (umbral 1.5 s)", pwm_cmp > 0, NULL); break;
    case 37700:
        check("Timeout de seguridad (>1.5 s sin tramas): PWM 0, EN = 0",
              pwm_cmp == 0 && ci->modo == CM_PARO && log_tiene("TIMEOUT DE SEGURIDAD", marca), NULL); break;
    case 40000: esp_activo = 1; break;
    case 40400: fc01(0, 3); break;
    case 40450:
        snprintf(det, sizeof det, "coils leidos 0x%02X", coils_vistos);
        check("Enlace de vuelta: FC01 muestra EN = 0 (espera EN:1 del HMI)", (coils_vistos & 1) == 0 && ci->modo == CM_PARO, det); break;
    case 40800:
        snprintf(det, sizeof det, "IR4 = 0x%02X", ir_vistos[4]);
        check("Bit 4 (timeout com.) apagado con el enlace de vuelta", (ir_vistos[4] & 0x10) == 0, det); break;
    case 41000: fc05(COIL_EN, 1); break;
    case 46000: sensor_desconectado = 1; marca = loglen; break;
    case 46800:
        snprintf(det, sizeof det, "IR4 = 0x%02X", ir_vistos[4]);
        check("Sensor sin eco: bit 0 (sensor OK) apagado", (ir_vistos[4] & 0x01) == 0, det); break;
    case 47200:
        check("Sensor perdido ~0.8 s: paro de seguridad (PWM 0, EN = 0)",
              pwm_cmp == 0 && ci->modo == CM_PARO && log_tiene("PARO DE SEGURIDAD", marca), NULL); break;
    case 48000: sensor_desconectado = 0; break;
    case 48500: fc05(COIL_EN, 1); break;
    case 52000: fc06(HR_SETPOINT, 1500); break;
    case 52050: check("SP:1500 -> excepcion 0x03 (valor invalido)", ult_exc == 3, NULL); break;
    case 52100: fc04(0, 8); break;
    case 52150: check("FC04 de 8 registros -> excepcion 0x02", ult_exc == 2, NULL); break;
    case 52180: { uint8_t f[10] = {1, 2, 0, 0, 0, 1}; enviar(f, 6, 1); } break;
    case 52230: check("FC02 (no soportada) -> excepcion 0x01", ult_exc == 1, NULL); break;
    case 52300: { uint8_t f[10] = {1, 4, 0, 0, 0, 7, 0x12, 0x34}; if (rs_h == rs_t) rs_h = rs_t = 0; memcpy(&rs_rx[rs_t], f, 8); rs_t += 8; } break;
    case 52800:
        snprintf(det, sizeof det, "IR6 = %u", ir_vistos[6]);
        check("Trama con CRC malo: no responde y cuenta en IR6", ir_vistos[6] == 1, det); break;
    case 53000: consola("sp 20"); break;
    case 53100: fc03(0, 1); break;
    case 53150: check("Consola 'sp 20' escribe HR0 = 200 (lo ve el maestro)", hr_visto == 200, NULL);
                iniciar_escalon(53.0, 20.0); break;
    case 60000: cerrar_escalon("Escalon desde consola 8 -> 20 cm"); break;
    case 60100: consola("a"); marca = loglen; break;
    case 75250:
        snprintf(det, sizeof det, "IR0 a mitad de rampa = %u", ir_vistos[0]);
        check("Rampa: IR0 sigue al setpoint de la rampa", ir_vistos[0] > 220 && ir_vistos[0] < 300, det); break;
    case 91000:
        snprintf(det, sizeof det, "sp %.1f, altura real %.1f", ci->sp, real.h);
        check("Fin de rampa: PID sostiene spMax", log_tiene("Fin de rampa", marca) && fabs(ci->sp - ci->spMax) < 0.1 && fabs(real.h - ci->spMax) < 1.0, det); break;
    case 92000: consola("u 52"); break;
    case 94000: consola("plot 1"); marca = loglen; break;
    case 94500: check("plot 1: lineas sp:,altura:,pwm: para el Serial Plotter", log_tiene("sp:", marca) && log_tiene(",pwm:52.0", marca), NULL); break;
    case 94600: consola("plot 0"); break;
    case 96000: consola("fuente sim"); en_sim = 1; pwm_max_sim = 0; break;
    case 98000: consola("sp 15"); break;
    case 104000:
        snprintf(det, sizeof det, "altura simulada %.1f cm, PWM real max %u", ci->altura, pwm_max_sim);
        check("Fuente sim: el PID controla la planta simulada, ventilador real en 0", fabs(ci->altura - 15.0) < 1.0 && pwm_max_sim == 0, det);
        break;
    case 105000: consola("fuente real"); en_sim = 0; break;
    case 106000: consola("dump"); break;
    default: break;
    }
}

/* =============================== main =============================== */
int main(void)
{
    uint64_t fin_us = 107000000ull;
    char det[128];

    flog = fopen("log_firmware.txt", "w");
    fcsv = fopen("traza.csv", "w");
    fprintf(fcsv, "t,sp,altura_medida,altura_real,duty,modo\n");
    planta_init(&real, 36.4f, 2024u);

    app_init();
    while (sim_us < fin_us) {
        uint64_t sig = sim_us + 100;
        if (pend_sube && t_sube <= sig) { sim_us = t_sube; lectura_us = sim_us; echo = 1; if (eco_fn) eco_fn(); pend_sube = 0; }
        if (!pend_sube && pend_baja && t_baja <= sig) { sim_us = t_baja; lectura_us = sim_us; echo = 0; if (eco_fn) eco_fn(); pend_baja = 0; }
        sim_us = sig; lectura_us = sim_us;
        if (sim_us % 1000 == 0) {
            uint32_t ms = (uint32_t)(sim_us / 1000);
            if (systick_fn) systick_fn();
            planta_avanzar(&real, (float)pwm_cmp * 100.0f / 960.0f, 1);
            revisar_respuesta();
            muestrear(ms);
            escenario(ms);
        }
        app_poll();
    }

    printf("\n  Resumen Modbus: %d sondeos FC04, %d respuestas OK, %d timeouts, %d CRC malos en respuestas\n",
           n_polls, n_resp_ok, n_timeouts, n_crc_resp_malo);
    snprintf(det, sizeof det, "max %.2f ms (contrato: < 100 ms)", lat_max_us / 1000.0);
    check("Tiempo de respuesta Modbus", lat_max_us < 10000, det);
    check("Ningun sondeo sin respuesta (con la ESP32 conectada)", n_timeouts == 0, NULL);
    printf("\n  TOTAL: %d pasan, %d fallan\n", n_pasa, n_falla);
    fclose(flog); fclose(fcsv);
    return n_falla ? 1 : 0;
}
