/*
 * esp32_modem/main.c — ESP32-S3 PSK31 Hayes-style modem.
 *
 * Architecture:
 *   UART RX → AT command parser (command mode)
 *          → psk_tx_write() (data mode)
 *   PSK31 RX char callback → UART TX
 *
 * Audio:
 *   TX: psk_tx_read() → I2S DAC output
 *   RX: I2S ADC input → float conversion → psk_rx_feed()
 *
 * This file contains the application logic.  The HAL layer (I2S, UART)
 * is provided by ESP-IDF and must be implemented in your board-specific code.
 * Replace the HAL stubs at the bottom with real ESP-IDF calls.
 *
 * +++ escape: same Hayes guard-time sequence as linux_tcpmodem.
 *
 * Build with ESP-IDF:
 *   idf.py set-target esp32s3
 *   idf.py build
 * The libpsk library should be added as a component; see CMakeLists.txt
 * in the project root for linking instructions.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "psk.h"
#include <string.h>
#include <stdio.h>

/* ── HAL stubs (replace with real ESP-IDF I2S / UART calls) ──────────────── */

/*
 * Read samples from ADC.  Returns number of float32 samples placed in buf.
 * Non-blocking: returns 0 if no data available.
 */
static int hal_audio_read(float *buf, int max_samples)
{
    (void)buf; (void)max_samples;
    /* Replace with: i2s_read(...) + int16→float conversion */
    return 0;
}

/*
 * Write samples to DAC.  Returns number of samples written.
 */
static int hal_audio_write(const float *buf, int samples)
{
    (void)buf; (void)samples;
    /* Replace with: float→int16 conversion + i2s_write(...) */
    return samples;
}

/*
 * Read bytes from UART.  Returns number of bytes placed in buf.
 * Non-blocking: returns 0 if no data.
 */
static int hal_uart_read(char *buf, int max_bytes)
{
    (void)buf; (void)max_bytes;
    /* Replace with: uart_read_bytes(UART_NUM_0, buf, max_bytes, 0) */
    return 0;
}

/*
 * Write bytes to UART.
 */
static void hal_uart_write(const char *buf, int len)
{
    (void)buf; (void)len;
    /* Replace with: uart_write_bytes(UART_NUM_0, buf, len) */
}

/* ── Modem state ──────────────────────────────────────────────────────────── */

#define SAMPLE_RATE       8000u
#define CARRIER_HZ        1500.0f
#define DEFAULT_PSK_MODE  PSK_MODE_BPSK31
#define AUDIO_BLK         64       /* samples per tick (8 ms) */
#define ESCAPE_GUARD      SAMPLE_RATE   /* 1 second in samples */
#define ESCAPE_COUNT      3

typedef enum { MODEM_IDLE, MODEM_DATA, MODEM_COMMAND } modem_state_t;

static psk_tx_t     *g_tx;
static psk_rx_t     *g_rx;
static modem_state_t g_state;
static psk_mode_t    g_mode;
static float         g_carrier_hz;

/* Escape detection */
static int g_esc_count;
static int g_esc_guard_before;   /* samples since last non-'+' */
static int g_esc_guard_after;    /* samples since completing +++ */

/* AT command buffer */
static char g_cmd_buf[128];
static int  g_cmd_len;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static void uart_send(const char *s)
{
    hal_uart_write(s, (int)strlen(s));
    hal_uart_write("\r\n", 2);
}

static void modem_create_psk(void)
{
    if (g_tx) psk_tx_destroy(g_tx);
    if (g_rx) psk_rx_destroy(g_rx);
    g_tx = psk_tx_create(SAMPLE_RATE, g_carrier_hz, g_mode);
    g_rx = psk_rx_create(SAMPLE_RATE, g_carrier_hz, g_mode);
}

static void on_rx_char(char c, void *ud)
{
    (void)ud;
    if (g_state == MODEM_DATA)
        hal_uart_write(&c, 1);
}

/* ── AT command parser ────────────────────────────────────────────────────── */

static const char *mode_names[] = {
    "BPSK31", "QPSK31", "BPSK63", "BPSK125", "QPSK63", "QPSK125"
};

static psk_mode_t parse_mode(const char *s)
{
    for (int i = 0; i < 6; i++)
        if (strcasecmp(s, mode_names[i]) == 0)
            return (psk_mode_t)i;
    int n = atoi(s);
    if (n >= 0 && n <= 5) return (psk_mode_t)n;
    return (psk_mode_t)-1;
}

static void handle_at(const char *cmd)
{
    char uc[128];
    int i;
    for (i = 0; cmd[i] && i < 127; i++)
        uc[i] = (cmd[i] >= 'a' && cmd[i] <= 'z') ? cmd[i] - 32 : cmd[i];
    uc[i] = '\0';

    const char *p = uc;
    if (p[0] == 'A' && p[1] == 'T') p += 2;

    if (*p == '\0') { uart_send("OK"); return; }
    if (strcmp(p, "Z") == 0) {
        g_mode = DEFAULT_PSK_MODE;
        g_carrier_hz = CARRIER_HZ;
        modem_create_psk();
        psk_rx_set_char_callback(g_rx, on_rx_char, NULL);
        uart_send("OK");
        return;
    }
    if (strcmp(p, "H") == 0) {
        g_state = MODEM_IDLE;
        uart_send("NO CARRIER");
        return;
    }
    if (strcmp(p, "A") == 0) {
        g_state = MODEM_DATA;
        g_esc_count = 0;
        g_esc_guard_before = ESCAPE_GUARD;
        uart_send("CONNECT");
        return;
    }
    if (strncmp(p, "+MODE=", 6) == 0) {
        psk_mode_t nm = parse_mode(p + 6);
        if ((int)nm < 0) { uart_send("ERROR"); return; }
        g_mode = nm;
        modem_create_psk();
        psk_rx_set_char_callback(g_rx, on_rx_char, NULL);
        uart_send("OK");
        return;
    }
    if (strncmp(p, "+FREQ=", 6) == 0) {
        float f = (float)atof(p + 6);
        if (f < 100.0f || f > 3500.0f) { uart_send("ERROR"); return; }
        g_carrier_hz = f;
        modem_create_psk();
        psk_rx_set_char_callback(g_rx, on_rx_char, NULL);
        uart_send("OK");
        return;
    }
    if (strcmp(p, "I") == 0) {
        uart_send("PSK31 ESP32 Modem v1.0");
        uart_send("OK");
        return;
    }
    uart_send("ERROR");
}

/* ── Main tick (call from FreeRTOS task or main loop) ────────────────────── */

void modem_tick(void)
{
    float   audio_in[AUDIO_BLK];
    float   audio_out[AUDIO_BLK];
    char    uart_in[64];
    int     uart_bytes, audio_in_count;

    /* Read and process UART */
    uart_bytes = hal_uart_read(uart_in, (int)sizeof(uart_in));
    for (int i = 0; i < uart_bytes; i++) {
        char c = uart_in[i];

        if (g_state == MODEM_DATA) {
            /* +++ escape detection */
            if (c != '+') {
                /* Flush any accumulated '+' */
                if (g_esc_count > 0) {
                    char plus[4];
                    memset(plus, '+', g_esc_count);
                    psk_tx_write(g_tx, plus, (size_t)g_esc_count);
                    g_esc_count = 0;
                }
                g_esc_guard_before = 0;
                psk_tx_write(g_tx, &c, 1);
            } else {
                if (g_esc_guard_before >= ESCAPE_GUARD) {
                    g_esc_count++;
                    if (g_esc_count == ESCAPE_COUNT)
                        g_esc_guard_after = 0;
                } else {
                    g_esc_count = 0;
                    psk_tx_write(g_tx, &c, 1);
                }
            }
        } else if (g_state == MODEM_COMMAND) {
            if (c == '\r' || c == '\n') {
                if (g_cmd_len > 0) {
                    g_cmd_buf[g_cmd_len] = '\0';
                    handle_at(g_cmd_buf);
                    g_cmd_len = 0;
                }
            } else if (g_cmd_len < 126) {
                g_cmd_buf[g_cmd_len++] = c;
            }
        }
    }

    /* Audio tick: advance guard timers */
    if (g_state == MODEM_DATA) {
        g_esc_guard_before += AUDIO_BLK;
        if (g_esc_guard_before > ESCAPE_GUARD * 2)
            g_esc_guard_before = ESCAPE_GUARD * 2;

        if (g_esc_count == ESCAPE_COUNT) {
            g_esc_guard_after += AUDIO_BLK;
            if (g_esc_guard_after >= ESCAPE_GUARD) {
                g_state = MODEM_COMMAND;
                g_esc_count = 0;
                uart_send("\r\nOK");
            }
        }
    }

    /* RX audio */
    audio_in_count = hal_audio_read(audio_in, AUDIO_BLK);
    if (audio_in_count > 0 && g_state == MODEM_DATA)
        psk_rx_feed(g_rx, audio_in, (size_t)audio_in_count);

    /* TX audio */
    psk_tx_read(g_tx, audio_out, AUDIO_BLK);
    hal_audio_write(audio_out, AUDIO_BLK);
}

/* ── Initialisation (call once at startup) ───────────────────────────────── */

void modem_init_esp32(void)
{
    g_mode       = DEFAULT_PSK_MODE;
    g_carrier_hz = CARRIER_HZ;
    g_state      = MODEM_COMMAND;
    g_esc_guard_before = ESCAPE_GUARD;

    modem_create_psk();
    psk_rx_set_char_callback(g_rx, on_rx_char, NULL);
    uart_send("PSK31 ESP32 MODEM READY");
}

/*
 * Minimal app_main — in real ESP-IDF this would create a FreeRTOS task.
 */
void app_main(void)
{
    modem_init_esp32();
    while (1) {
        modem_tick();
        /* Replace with: vTaskDelay(pdMS_TO_TICKS(8)); */
    }
}
