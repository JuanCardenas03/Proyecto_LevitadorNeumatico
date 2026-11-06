/* ============================================================================
 * consola.c  -  Comandos por el puerto de depuracion (115200, CR o LF)
 *
 * Los comandos de control son los mismos del monitor serie de la ESP32-PID
 * (sp=, u=, ff=, kp=, ki=, kd=, fl=, a, b, c, d, p). Se aceptan "sp=15" y
 * "sp 15". Escriben los MISMOS registros que escribe el maestro Modbus, asi
 * que el HMI ve los cambios.
 * ==========================================================================*/
#include "project.h"
#include "consola.h"
#include "debug_log.h"
#include "control.h"
#include <string.h>
#include <ctype.h>

#define LINEA_MAX 48u

static char    linea[LINEA_MAX];
static uint8_t n_linea;

void consola_ayuda(void)
{
    dbg_puts("Comandos de control:\r\n"
             "  sp 15 | 15 | sp 50%  setpoint (cm o % de la altura util); EN=1, modo auto\r\n"
             "  u 51          MANUAL con duty en % (buscar el duty de flote)\r\n"
             "  ff 49         feedforward de sostenimiento (%); solo cambia por aqui\r\n"
             "  kp 1.0 | ki 0.6 | kd 0.05  ganancias (% de duty por cm)\r\n"
             "  fl 20         piso de duty en vuelo (%)\r\n"
             "  p             paro (EN=0)      rst   reinicia el integrador\r\n"
             "  en 0|1  emg 0|1  modo 0|1     coils / modo, igual que el HMI\r\n"
             "  a             rampa del sp hasta el maximo en 30 s\r\n"
             "  b             barrido PWM 0->100 % en 30 s (lazo abierto)\r\n"
             "  c             recalibrar el fondo (pelota abajo, ventilador apagado)\r\n"
             "  plot 0|1      salida para Serial Plotter (sp,altura,pwm)   d  +distancia\r\n"
             "  fuente real|sim  sensor real o planta simulada (ventilador apagado)\r\n"
             "Comunicacion:\r\n"
             "  etapa 0|1  log 0|1|2  mute 0|1  dump  reset  help\r\n");
}

void consola_dump(const app_cfg_t *cfg, const mb_data_t *d)
{
    const ctrl_info_t *ci = ctrl_info();
    uint16_t i;

    dbg_puts("---- Holding registers ----\r\n");
    for (i = 0u; i < MB_NUM_HR; i++) {
        dbg_printf("  HR%u %-12s = %-10s [%u]\r\n", i, nombre_hr(i),
                   fmt_hr(i, d->hr[i]), d->hr[i]);
    }
    dbg_puts("---- Input registers ----\r\n");
    for (i = 0u; i < MB_NUM_IR; i++) {
        dbg_printf("  IR%u %-16s = %-10s [%u]\r\n", i, nombre_ir(i),
                   fmt_ir(i, d->ir[i]), d->ir[i]);
    }
    dbg_puts("---- Coils ----\r\n");
    for (i = 0u; i < MB_NUM_COILS; i++) {
        dbg_printf("  C%u  %-17s = %s\r\n", i, nombre_coil(i),
                   (d->coil[i] != 0u) ? "ON" : "OFF");
    }
    dbg_puts("---- Control ----\r\n");
    dbg_printf("  modo=%s  sp=%s cm  altura=%s cm  duty=%s %%  I=%s %%\r\n",
               ctrl_nombre_modo(ci->modo),
               fx(ctrl_redondear(ci->sp * 100.0f), 2u),
               fx(ctrl_redondear(ci->altura * 10.0f), 1u),
               fx(ctrl_redondear(ci->duty * 10.0f), 1u),
               fx(ctrl_redondear(ci->I * 100.0f), 2u));
    dbg_printf("  uFF=%s %%  piso=%s %%  base=%s cm  spMax=%s cm  dist=%s cm  fallos=%u\r\n",
               fx(ctrl_redondear(ci->uFF * 10.0f), 1u),
               fx(ctrl_redondear(ci->piso * 10.0f), 1u),
               fx(ctrl_redondear(ci->base * 10.0f), 1u),
               fx(ctrl_redondear(ci->spMax * 10.0f), 1u),
               fx(ctrl_redondear(ci->dist * 10.0f), 1u),
               ci->fallos);
    dbg_printf("---- etapa=%u log=%u mute=%u fuente=%s plot=%u | pulsos RST=%u\r\n",
               cfg->etapa, cfg->log_nivel, cfg->mute,
               (cfg->fuente == FUENTE_SIM) ? "sim" : "real", cfg->plot,
               ci->pulsos_rst);
}

/* Numero decimal ("15", "-3.5", "0.25"). Sin strtod: en newlib usa el heap. */
static uint8_t leer_num(const char *s, float *out)
{
    float   v = 0.0f, escala = 1.0f;
    uint8_t neg = 0u, hay = 0u;

    if (s == NULL) {
        return 0u;
    }
    if (*s == '-') { neg = 1u; s++; } else if (*s == '+') { s++; }
    while (isdigit((unsigned char)*s)) {
        v = v * 10.0f + (float)(*s - '0');
        hay = 1u;
        s++;
    }
    if (*s == '.') {
        s++;
        while (isdigit((unsigned char)*s)) {
            escala *= 0.1f;
            v += (float)(*s - '0') * escala;
            hay = 1u;
            s++;
        }
    }
    *out = (neg != 0u) ? -v : v;
    return hay;
}

static float lim(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void cmd_sp(mb_data_t *d, const char *arg, float x)
{
    const ctrl_info_t *ci = ctrl_info();
    float cm = (strchr(arg, '%') != NULL) ? lim(x, 0.0f, 100.0f) * ci->spMax / 100.0f : x;
    uint16_t hr0 = (uint16_t)ctrl_redondear(lim(cm, 0.0f, 100.0f) * 10.0f);

    d->hr[HR_SETPOINT] = hr0;
    d->hr[HR_MODO]     = 1u;
    d->coil[COIL_EN]   = 1u;
    mb_sync(d);
    dbg_printf("[CON] SP = %s cm (HR0 = %u), modo auto, EN = 1%s\r\n",
               fx(hr0, 1u), hr0,
               (cm > ci->spMax) ? " -> fuera del rango util, el PID usa el maximo" : "");
}

static void cmd_ganancia(mb_data_t *d, uint16_t hr, const char *nombre, float x)
{
    d->hr[hr] = (uint16_t)ctrl_redondear(lim(x, 0.0f, 655.35f) * 100.0f);
    dbg_printf("[CON] %s = %s (HR%u = %u)\r\n", nombre, fx(d->hr[hr], 2u), hr, d->hr[hr]);
}

static void ejecutar(app_cfg_t *cfg, mb_data_t *d)
{
    const char *cmd = strtok(linea, " \t=");
    const char *arg = strtok(NULL, " \t=");
    float   x = 0.0f;
    uint8_t hay;

    if (cmd == NULL) {
        return;
    }
    /* Un numero suelto es un setpoint (igual que en la ESP32) */
    if (isdigit((unsigned char)cmd[0]) || (cmd[0] == '.')) {
        arg = cmd;
        cmd = "sp";
    }
    hay = leer_num(arg, &x);

    if ((strcmp(cmd, "help") == 0) || (strcmp(cmd, "?") == 0)) {
        consola_ayuda();
    } else if (strcmp(cmd, "dump") == 0) {
        consola_dump(cfg, d);

    /* ----- control ----- */
    } else if ((strcmp(cmd, "sp") == 0) && hay) {
        cmd_sp(d, arg, x);
    } else if ((strcmp(cmd, "u") == 0) && hay) {
        d->hr[HR_PWM_MANUAL] = (uint16_t)ctrl_redondear(lim(x, 0.0f, 100.0f) * 10.0f);
        d->hr[HR_MODO]       = 0u;
        d->coil[COIL_EN]     = 1u;
        dbg_printf("[CON] MANUAL, duty = %s %% (HR5 = %u), EN = 1\r\n",
                   fx(d->hr[HR_PWM_MANUAL], 1u), d->hr[HR_PWM_MANUAL]);
    } else if ((strcmp(cmd, "ff") == 0) && hay) {
        ctrl_set_uff(x);
        dbg_printf("[CON] Feedforward uFF = %s %%\r\n", fx(ctrl_redondear(ctrl_info()->uFF * 10.0f), 1u));
    } else if ((strcmp(cmd, "fl") == 0) && hay) {
        ctrl_set_piso(x);
        dbg_printf("[CON] Piso de duty = %s %%\r\n", fx(ctrl_redondear(ctrl_info()->piso * 10.0f), 1u));
    } else if ((strcmp(cmd, "kp") == 0) && hay) {
        cmd_ganancia(d, HR_KP, "Kp", x);
    } else if ((strcmp(cmd, "ki") == 0) && hay) {
        cmd_ganancia(d, HR_KI, "Ki", x);
    } else if ((strcmp(cmd, "kd") == 0) && hay) {
        cmd_ganancia(d, HR_KD, "Kd", x);
    } else if ((strcmp(cmd, "p") == 0) || (strcmp(cmd, "paro") == 0)) {
        d->coil[COIL_EN] = 0u;
        dbg_puts("[CON] PARO (EN = 0)\r\n");
    } else if ((strcmp(cmd, "en") == 0) && hay) {
        d->coil[COIL_EN] = (x != 0.0f) ? 1u : 0u;
        dbg_printf("[CON] EN = %u\r\n", d->coil[COIL_EN]);
    } else if ((strcmp(cmd, "emg") == 0) && hay) {
        d->coil[COIL_EMG] = (x != 0.0f) ? 1u : 0u;
        dbg_printf("[CON] EMG = %u\r\n", d->coil[COIL_EMG]);
    } else if ((strcmp(cmd, "modo") == 0) && hay) {
        d->hr[HR_MODO] = (x != 0.0f) ? 1u : 0u;
        dbg_printf("[CON] Modo = %s\r\n", d->hr[HR_MODO] ? "auto" : "manual");
    } else if (strcmp(cmd, "rst") == 0) {
        ctrl_reset_integrador();
    } else if ((strcmp(cmd, "a") == 0) || (strcmp(cmd, "rampa") == 0)) {
        d->hr[HR_MODO]   = 1u;
        d->coil[COIL_EN] = 1u;
        ctrl_pedir_rampa();
        dbg_printf("[CON] RAMPA sp -> %s cm en 30 s\r\n",
                   fx(ctrl_redondear(ctrl_info()->spMax * 10.0f), 1u));
    } else if ((strcmp(cmd, "b") == 0) || (strcmp(cmd, "barrido") == 0)) {
        d->coil[COIL_EN] = 1u;
        ctrl_pedir_barrido();
        dbg_puts("[CON] BARRIDO PWM 0 -> 100 % en 30 s (caracterizacion)\r\n");
    } else if ((strcmp(cmd, "c") == 0) || (strcmp(cmd, "cal") == 0)) {
        d->coil[COIL_EN] = 0u;
        ctrl_calibrar(1500u);
        dbg_puts("[CON] Calibrando fondo (pelota abajo, ventilador apagado)...\r\n");
    } else if ((strcmp(cmd, "plot") == 0) && hay) {
        cfg->plot = (x != 0.0f) ? 1u : 0u;
    } else if (strcmp(cmd, "d") == 0) {
        cfg->plot_dist = (cfg->plot_dist != 0u) ? 0u : 1u;
    } else if ((strcmp(cmd, "fuente") == 0) && (arg != NULL)) {
        cfg->fuente = (strcmp(arg, "sim") == 0) ? FUENTE_SIM : FUENTE_REAL;
        cfg->cambio_fuente = 1u;
        dbg_printf("[CON] Fuente = %s (EN = 0, se recalibra)\r\n",
                   (cfg->fuente == FUENTE_SIM) ? "planta SIMULADA, ventilador apagado" : "HC-SR04 real");

    /* ----- comunicacion ----- */
    } else if ((strcmp(cmd, "etapa") == 0) && hay) {
        cfg->etapa = (x != 0.0f) ? 1u : 0u;
        dbg_printf("[CFG] etapa = %u (%s)\r\n", cfg->etapa,
                   cfg->etapa ? "esclavo Modbus" : "sniffer pasivo, no responde");
    } else if ((strcmp(cmd, "log") == 0) && hay) {
        cfg->log_nivel = (x < 0.5f) ? 0u : ((x > 1.5f) ? 2u : 1u);
        dbg_printf("[CFG] log = %u\r\n", cfg->log_nivel);
    } else if ((strcmp(cmd, "mute") == 0) && hay) {
        cfg->mute = (x != 0.0f) ? 1u : 0u;
        dbg_printf("[CFG] mute = %u%s\r\n", cfg->mute,
                   cfg->mute ? " (la PSoC NO responde: la ESP32 vera timeout 0xE2)" : "");
    } else if (strcmp(cmd, "reset") == 0) {
        d->tramas_ok   = 0u;
        d->errores_crc = 0u;
        dbg_puts("[CFG] contadores en cero\r\n");
    } else {
        dbg_printf("Comando no reconocido o falta el valor: '%s'. Escriba help\r\n", cmd);
    }
    mb_sync(d);
}

void consola_poll(app_cfg_t *cfg, mb_data_t *d)
{
    char c;

    while (UART_DBG_GetRxBufferSize() != 0u) {
        c = (char)UART_DBG_ReadRxData();

        if ((c == '\r') || (c == '\n')) {
            if (n_linea > 0u) {
                linea[n_linea] = '\0';
                dbg_puts("\r\n");
                ejecutar(cfg, d);
                n_linea = 0u;
            }
        } else if ((c == '\b') || (c == 0x7F)) {
            if (n_linea > 0u) {
                n_linea--;
                dbg_puts("\b \b");
            }
        } else if ((n_linea < (LINEA_MAX - 1u)) && isprint((unsigned char)c)) {
            linea[n_linea++] = (char)tolower((unsigned char)c);
            UART_DBG_PutChar(c);   /* eco de lo que se escribe */
        }
    }
}