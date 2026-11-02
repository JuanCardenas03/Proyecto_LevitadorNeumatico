/* ============================================================================
 * consola.c  -  Comandos por el puerto de depuracion
 *
 * Escriba en el monitor serie (115200, fin de linea CR o LF):
 *   help            lista de comandos
 *   etapa 0|1       0 = sniffer pasivo, 1 = esclavo Modbus
 *   log 0|1|2       0 silencio, 1 escrituras + resumen/s, 2 todo en hex
 *   alt <v>         fija la altura (cm x 10). Ej: alt 226 -> 22.6 cm
 *   err <v>         fuerza el error (cm x 10, con signo). Ej: err -35
 *   est <v>         fuerza la palabra de estado. Ej: est 0x21
 *   sim             quita todo lo forzado (vuelve la planta simulada)
 *   mute 0|1        1 = la PSoC deja de responder (enlace caido)
 *   dump            imprime todos los registros
 *   reset           pone en cero los contadores de tramas y CRC
 * ==========================================================================*/
#include "project.h"
#include "consola.h"
#include "debug_log.h"
#include "plant_sim.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define LINEA_MAX 48u

static char    linea[LINEA_MAX];
static uint8_t n_linea;

void consola_ayuda(void)
{
    dbg_puts("Comandos:\r\n"
             "  etapa 0|1   0 = sniffer pasivo, 1 = esclavo Modbus\r\n"
             "  log 0|1|2   0 silencio, 1 escrituras + resumen/s, 2 todo en hex\r\n"
             "  alt <v>     fija altura (cm x 10)       ej: alt 226\r\n"
             "  err <v>     fuerza error (cm x 10)      ej: err -35\r\n"
             "  est <v>     fuerza palabra de estado    ej: est 0x21\r\n"
             "  sim         quita lo forzado (planta simulada)\r\n"
             "  mute 0|1    1 = no responder (simula enlace caido)\r\n"
             "  dump        imprime todos los registros\r\n"
             "  reset       contadores de tramas y CRC a cero\r\n"
             "  help        esta ayuda\r\n");
}

void consola_dump(const app_cfg_t *cfg, const mb_data_t *d)
{
    const plant_override_t *ov = plant_overrides();
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
    dbg_printf("---- etapa=%u log=%u mute=%u | forzado: alt=%s err=%s est=%s"
               " | pulsos RST=%u\r\n",
               cfg->etapa, cfg->log_nivel, cfg->mute,
               ov->alt_activa ? fx(ov->alt_valor, 1u) : "no",
               ov->err_activo ? fx(ov->err_valor, 1u) : "no",
               ov->est_activo ? bin6(ov->est_valor) : "no",
               plant_resets_integrador());
}

static void ejecutar(app_cfg_t *cfg, mb_data_t *d)
{
    plant_override_t *ov = plant_overrides();
    char *cmd = strtok(linea, " \t");
    char *arg = strtok(NULL, " \t");
    char *fin = NULL;
    long  v   = 0;
    uint8_t hay_valor = 0u;

    if (cmd == NULL) {
        return;
    }
    if (arg != NULL) {
        v = strtol(arg, &fin, 0);       /* acepta decimal, 0x.. y negativos */
        hay_valor = (fin != arg) ? 1u : 0u;
    }

    if ((strcmp(cmd, "help") == 0) || (strcmp(cmd, "?") == 0)) {
        consola_ayuda();
    } else if (strcmp(cmd, "dump") == 0) {
        consola_dump(cfg, d);
    } else if ((strcmp(cmd, "etapa") == 0) && hay_valor) {
        cfg->etapa = (v != 0) ? 1u : 0u;
        dbg_printf("[CFG] etapa = %u (%s)\r\n", cfg->etapa,
                   cfg->etapa ? "esclavo Modbus" : "sniffer pasivo, no responde");
    } else if ((strcmp(cmd, "log") == 0) && hay_valor) {
        cfg->log_nivel = (v < 0) ? 0u : ((v > 2) ? 2u : (uint8_t)v);
        dbg_printf("[CFG] log = %u\r\n", cfg->log_nivel);
    } else if ((strcmp(cmd, "mute") == 0) && hay_valor) {
        cfg->mute = (v != 0) ? 1u : 0u;
        dbg_printf("[CFG] mute = %u%s\r\n", cfg->mute,
                   cfg->mute ? " (la PSoC NO responde: la ESP32 vera timeout 0xE2)" : "");
    } else if ((strcmp(cmd, "alt") == 0) && hay_valor) {
        ov->alt_activa = 1u;
        ov->alt_valor  = (int16_t)v;
        dbg_printf("[SIM] altura fija = %s cm\r\n", fx(ov->alt_valor, 1u));
    } else if ((strcmp(cmd, "err") == 0) && hay_valor) {
        ov->err_activo = 1u;
        ov->err_valor  = (int16_t)v;
        dbg_printf("[SIM] error forzado = %s cm\r\n", fx(ov->err_valor, 1u));
    } else if ((strcmp(cmd, "est") == 0) && hay_valor) {
        ov->est_activo = 1u;
        ov->est_valor  = (uint16_t)(v & 0x3F);
        dbg_printf("[SIM] estado forzado = 0b%s\r\n", bin6(ov->est_valor));
    } else if (strcmp(cmd, "sim") == 0) {
        plant_clear_overrides();
        dbg_puts("[SIM] planta simulada (sin valores forzados)\r\n");
    } else if (strcmp(cmd, "reset") == 0) {
        d->tramas_ok   = 0u;
        d->errores_crc = 0u;
        mb_sync(d);
        dbg_puts("[CFG] contadores en cero\r\n");
    } else {
        dbg_printf("Comando no reconocido o falta el valor: '%s'. Escriba help\r\n", cmd);
    }
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
