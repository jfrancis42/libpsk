/*
 * psk_tcpmodem — Linux PSK31 Hayes-style modem over TCP.
 *
 * Emulates a Hayes (AT command) modem:
 *   - Listens on a TCP port (default 7300)
 *   - In DATA mode: TCP→PSK31 TX, PSK31 RX→TCP
 *   - "+++" with >1s guard time on either side → switches to COMMAND mode
 *   - AT commands configure mode, carrier frequency, sample rate, etc.
 *
 * Audio: reads/writes raw float32 PCM files (or stdin/stdout) at 8 kHz.
 *        For real hardware, pipe through sox or ALSA aplay/arecord.
 *
 * Usage:
 *   psk_tcpmodem [--port PORT] [--carrier HZ] [--mode MODE]
 *                [--rx-file FILE] [--tx-file FILE]
 *
 *   Default: port=7300, carrier=1500Hz, mode=BPSK31
 *   --rx-file: read PCM float32 8kHz from FILE (default: stdin)
 *   --tx-file: write PCM float32 8kHz to FILE  (default: stdout)
 *
 * This is a synchronous single-threaded design using select() for I/O
 * multiplexing between the TCP socket, RX audio, and TX audio.
 *
 * MIT License. Copyright (c) 2025.
 */

#define _POSIX_C_SOURCE 200809L
#include "psk.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>

/* ── Configuration ────────────────────────────────────────────────────────── */

#define DEFAULT_PORT     7300
#define DEFAULT_CARRIER  1500.0f
#define DEFAULT_MODE     PSK_MODE_BPSK31
#define SAMPLE_RATE      8000u
#define AUDIO_BLK        256     /* samples per audio I/O block */
#define TCP_BUF          4096    /* TCP receive buffer size */
#define TX_TEXT_MAX      1024    /* max text queued for TX */

/* Guard time for +++ escape: 1 second */
#define ESCAPE_GUARD_SAMPLES  (SAMPLE_RATE * 1)
#define ESCAPE_CHAR           '+'
#define ESCAPE_COUNT          3

static const char *mode_names[] = {
    "BPSK31", "QPSK31", "BPSK63", "BPSK125", "QPSK63", "QPSK125"
};

/* ── State ─────────────────────────────────────────────────────────────────── */

typedef enum {
    MODEM_IDLE,
    MODEM_DATA,
    MODEM_COMMAND,
} modem_state_t;

typedef struct {
    /* TCP */
    int      listen_fd;
    int      client_fd;

    /* Audio FDs */
    int      rx_fd;   /* read raw float32 PCM */
    int      tx_fd;   /* write raw float32 PCM */

    /* Modem state */
    modem_state_t state;
    psk_mode_t    psk_mode;
    float         carrier_hz;
    psk_tx_t     *tx;
    psk_rx_t     *rx;

    /* +++ escape detection */
    int    esc_count;         /* '+' chars accumulated */
    int    esc_guard_after;   /* samples since last non-'+' char (> ESCAPE_GUARD_SAMPLES = ok) */
    int    esc_guard_before;  /* samples since last +++ sequence start */
    int    esc_silence_before;/* samples of silence before first '+' */

    /* AT command buffer */
    char   cmd_buf[256];
    int    cmd_len;

    /* RX char buffer → TCP */
    char   rx_text[TCP_BUF];
    int    rx_text_len;
} modem_t;

static volatile int g_running = 1;

/* ── Utilities ─────────────────────────────────────────────────────────────── */

static void log_msg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void tcp_send(modem_t *m, const char *s, int len)
{
    if (m->client_fd < 0 || len <= 0) return;
    if (len < 0) len = (int)strlen(s);
    int sent = 0;
    while (sent < len) {
        int r = (int)write(m->client_fd, s + sent, (size_t)(len - sent));
        if (r <= 0) { close(m->client_fd); m->client_fd = -1; break; }
        sent += r;
    }
}

static void tcp_puts(modem_t *m, const char *s)
{
    tcp_send(m, s, (int)strlen(s));
    tcp_send(m, "\r\n", 2);
}

/* ── Modem creation / destruction ──────────────────────────────────────────── */

static void modem_create_psk(modem_t *m)
{
    if (m->tx) { psk_tx_destroy(m->tx); m->tx = NULL; }
    if (m->rx) { psk_rx_destroy(m->rx); m->rx = NULL; }
    m->tx = psk_tx_create(SAMPLE_RATE, m->carrier_hz, m->psk_mode);
    m->rx = psk_rx_create(SAMPLE_RATE, m->carrier_hz, m->psk_mode);
}

/* RX character callback — buffers for sending to TCP client */
static void on_rx_char(char c, void *ud)
{
    modem_t *m = (modem_t *)ud;
    if (m->state != MODEM_DATA || m->client_fd < 0) return;
    /* Echo received character directly to TCP */
    write(m->client_fd, &c, 1);
}

static void modem_init(modem_t *m, int port, int rx_fd, int tx_fd)
{
    memset(m, 0, sizeof(*m));
    m->client_fd = -1;
    m->rx_fd     = rx_fd;
    m->tx_fd     = tx_fd;
    m->psk_mode  = DEFAULT_MODE;
    m->carrier_hz = DEFAULT_CARRIER;
    m->state     = MODEM_IDLE;
    m->esc_silence_before = ESCAPE_GUARD_SAMPLES; /* pre-loaded: start OK */

    modem_create_psk(m);
    psk_rx_set_char_callback(m->rx, on_rx_char, m);

    /* Create listening socket */
    m->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m->listen_fd < 0) { perror("socket"); exit(1); }
    int yes = 1;
    setsockopt(m->listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);
    if (bind(m->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); exit(1);
    }
    listen(m->listen_fd, 1);
    log_msg("PSK modem listening on port %d (mode=%s, carrier=%.0f Hz)",
            port, mode_names[m->psk_mode], m->carrier_hz);
}

/* ── AT command parser ─────────────────────────────────────────────────────── */

static psk_mode_t parse_mode(const char *s)
{
    for (int i = 0; i < 6; i++)
        if (strcasecmp(s, mode_names[i]) == 0)
            return (psk_mode_t)i;
    /* Also accept numeric */
    int n = atoi(s);
    if (n >= 0 && n <= 5) return (psk_mode_t)n;
    return (psk_mode_t)-1;
}

static void handle_at(modem_t *m, const char *cmd)
{
    /* Uppercase the copy for easier parsing */
    char uc[256];
    int i;
    for (i = 0; cmd[i] && i < 255; i++)
        uc[i] = (cmd[i] >= 'a' && cmd[i] <= 'z') ? cmd[i] - 32 : cmd[i];
    uc[i] = '\0';

    log_msg("AT: %s", cmd);

    /* Strip leading "AT" */
    const char *p = uc;
    if (p[0] == 'A' && p[1] == 'T') p += 2;

    if (*p == '\0' || strcmp(p, "") == 0) {
        /* bare "AT" */
        tcp_puts(m, "OK");
        return;
    }
    if (strcmp(p, "Z") == 0) {
        /* ATZ — reset */
        m->psk_mode  = DEFAULT_MODE;
        m->carrier_hz = DEFAULT_CARRIER;
        modem_create_psk(m);
        psk_rx_set_char_callback(m->rx, on_rx_char, m);
        tcp_puts(m, "OK");
        return;
    }
    if (strcmp(p, "H") == 0 || strcmp(p, "H0") == 0) {
        /* ATH — hang up */
        m->state = MODEM_IDLE;
        tcp_puts(m, "NO CARRIER");
        return;
    }
    if (strcmp(p, "A") == 0) {
        /* ATA — go to data mode */
        m->state = MODEM_DATA;
        m->esc_count = 0;
        m->esc_silence_before = ESCAPE_GUARD_SAMPLES;
        tcp_puts(m, "CONNECT");
        return;
    }
    if (strncmp(p, "+MODE=", 6) == 0) {
        psk_mode_t nm = parse_mode(p + 6);
        if ((int)nm < 0) { tcp_puts(m, "ERROR"); return; }
        m->psk_mode = nm;
        modem_create_psk(m);
        psk_rx_set_char_callback(m->rx, on_rx_char, m);
        tcp_puts(m, "OK");
        return;
    }
    if (strncmp(p, "+FREQ=", 6) == 0) {
        float f = (float)atof(p + 6);
        if (f < 100.0f || f > 3500.0f) { tcp_puts(m, "ERROR"); return; }
        m->carrier_hz = f;
        modem_create_psk(m);
        psk_rx_set_char_callback(m->rx, on_rx_char, m);
        tcp_puts(m, "OK");
        return;
    }
    if (strncmp(p, "+MODE?", 6) == 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "+MODE: %s", mode_names[m->psk_mode]);
        tcp_puts(m, buf);
        tcp_puts(m, "OK");
        return;
    }
    if (strncmp(p, "+FREQ?", 6) == 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "+FREQ: %.1f", m->carrier_hz);
        tcp_puts(m, buf);
        tcp_puts(m, "OK");
        return;
    }
    if (strcmp(p, "I") == 0) {
        tcp_puts(m, "PSK31 Hayes Modem v1.0");
        tcp_puts(m, "OK");
        return;
    }

    tcp_puts(m, "ERROR");
}

/* ── +++ escape detection ─────────────────────────────────────────────────── */

/*
 * Process a character received from the TCP client while in DATA mode.
 * Returns 1 if the character should be passed through to TX, 0 if consumed
 * by the escape detector.
 *
 * Hayes +++ escape sequence:
 *   1. No data for > 1 second (guard before)
 *   2. Exactly three '+' characters
 *   3. No data for > 1 second (guard after)
 */
static int esc_process_char(modem_t *m, char c)
{
    /* We count silence time via audio sample count (approx) */
    if (c != ESCAPE_CHAR) {
        /* Non-'+' character: reset escape sequence */
        if (m->esc_count > 0) {
            /* These were real '+' data chars — flush them to TX */
            char plus_str[4];
            memset(plus_str, ESCAPE_CHAR, m->esc_count);
            psk_tx_write(m->tx, plus_str, (size_t)m->esc_count);
            m->esc_count = 0;
        }
        m->esc_silence_before = 0;  /* reset guard before */
        return 1;  /* pass through */
    }

    /* It's a '+' */
    if (m->esc_silence_before >= (int)ESCAPE_GUARD_SAMPLES) {
        /* Guard-before satisfied */
        m->esc_count++;
        if (m->esc_count == ESCAPE_COUNT) {
            /* Full +++ seen; wait for guard-after (handled in audio tick) */
            m->esc_guard_after = 0;
            return 0;
        }
        return 0;  /* accumulating */
    }

    /* Guard not satisfied: pass '+' through */
    m->esc_count = 0;
    return 1;
}

/*
 * Called once per audio block (~32 ms at 256 samples/8000).
 * Tracks silence for guard-before timing and fires the escape if guard-after
 * has elapsed after receiving +++.
 */
static void esc_audio_tick(modem_t *m, int samples)
{
    if (m->state != MODEM_DATA) return;

    if (m->esc_count < ESCAPE_COUNT) {
        /* Accumulating guard-before time */
        m->esc_silence_before += samples;
        if (m->esc_silence_before > (int)(ESCAPE_GUARD_SAMPLES * 2))
            m->esc_silence_before = (int)(ESCAPE_GUARD_SAMPLES * 2);
    } else {
        /* Waiting for guard-after */
        m->esc_guard_after += samples;
        if (m->esc_guard_after >= (int)ESCAPE_GUARD_SAMPLES) {
            /* +++ complete! Switch to command mode */
            m->state     = MODEM_COMMAND;
            m->esc_count = 0;
            tcp_puts(m, "\r\nOK");
            log_msg("Escaped to command mode");
        }
    }
}

/* ── Main loop ──────────────────────────────────────────────────────────────── */

static void sig_handler(int sig) { (void)sig; g_running = 0; }

int main(int argc, char **argv)
{
    int port   = DEFAULT_PORT;
    float freq = DEFAULT_CARRIER;
    psk_mode_t pmode = DEFAULT_MODE;
    int rx_fd = STDIN_FILENO;
    int tx_fd = STDOUT_FILENO;

    /* Simple argument parsing */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i+1 < argc)
            port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--carrier") == 0 && i+1 < argc)
            freq = (float)atof(argv[++i]);
        else if (strcmp(argv[i], "--mode") == 0 && i+1 < argc) {
            psk_mode_t m = parse_mode(argv[++i]);
            if ((int)m >= 0) pmode = m;
        }
        else if (strcmp(argv[i], "--rx-file") == 0 && i+1 < argc) {
            rx_fd = open(argv[++i], O_RDONLY);
            if (rx_fd < 0) { perror("open rx-file"); exit(1); }
        }
        else if (strcmp(argv[i], "--tx-file") == 0 && i+1 < argc) {
            tx_fd = open(argv[++i], O_WRONLY|O_CREAT|O_TRUNC, 0644);
            if (tx_fd < 0) { perror("open tx-file"); exit(1); }
        }
        else if (strcmp(argv[i], "--help") == 0) {
            fprintf(stderr,
                "Usage: psk_tcpmodem [--port PORT] [--carrier HZ] [--mode MODE]\n"
                "                    [--rx-file FILE] [--tx-file FILE]\n"
                "Modes: BPSK31 QPSK31 BPSK63 BPSK125 QPSK63 QPSK125 (or 0-5)\n");
            return 0;
        }
    }

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    modem_t m;
    modem_init(&m, port, rx_fd, tx_fd);
    m.psk_mode  = pmode;
    m.carrier_hz = freq;
    modem_create_psk(&m);
    psk_rx_set_char_callback(m.rx, on_rx_char, &m);

    float  rx_audio[AUDIO_BLK];
    float  tx_audio[AUDIO_BLK];

    while (g_running) {
        /* Build poll set */
        struct pollfd pfds[3];
        int nfds = 0;

        /* Always listen for new connections if no client */
        int listen_idx = -1, client_idx = -1, audio_idx = -1;

        if (m.client_fd < 0) {
            pfds[nfds].fd     = m.listen_fd;
            pfds[nfds].events = POLLIN;
            listen_idx        = nfds++;
        }
        if (m.client_fd >= 0) {
            pfds[nfds].fd     = m.client_fd;
            pfds[nfds].events = POLLIN;
            client_idx        = nfds++;
        }
        /* Audio RX (non-blocking) */
        pfds[nfds].fd     = m.rx_fd;
        pfds[nfds].events = POLLIN;
        audio_idx         = nfds++;

        int ret = poll(pfds, (nfds_t)nfds, 50);  /* 50 ms timeout */
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        /* Accept new TCP connection */
        if (listen_idx >= 0 && (pfds[listen_idx].revents & POLLIN)) {
            struct sockaddr_in ca;
            socklen_t calen = sizeof(ca);
            int cfd = accept(m.listen_fd, (struct sockaddr *)&ca, &calen);
            if (cfd >= 0) {
                int flag = 1;
                setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
                m.client_fd = cfd;
                log_msg("Client connected from %s", inet_ntoa(ca.sin_addr));
                m.state = MODEM_COMMAND;
                tcp_puts(&m, "PSK31 MODEM READY");
                tcp_puts(&m, "");
                tcp_puts(&m, "AT+MODE=BPSK31 to change mode");
                tcp_puts(&m, "ATA to connect (data mode)");
                tcp_puts(&m, "");
            }
        }

        /* TCP data from client */
        if (client_idx >= 0 && (pfds[client_idx].revents & (POLLIN|POLLHUP))) {
            char buf[TCP_BUF];
            int r = (int)read(m.client_fd, buf, sizeof(buf));
            if (r <= 0) {
                log_msg("Client disconnected");
                close(m.client_fd);
                m.client_fd = -1;
                m.state     = MODEM_IDLE;
            } else {
                if (m.state == MODEM_DATA) {
                    /* DATA mode: detect +++ escape, send rest to TX */
                    for (int i = 0; i < r; i++) {
                        char c = buf[i];
                        if (m.esc_count == ESCAPE_COUNT) {
                            /* In guard-after period — discard */
                        } else if (esc_process_char(&m, c)) {
                            /* Pass to TX */
                            psk_tx_write(m.tx, &c, 1);
                        }
                    }
                } else if (m.state == MODEM_COMMAND) {
                    /* COMMAND mode: collect AT commands */
                    for (int i = 0; i < r; i++) {
                        char c = buf[i];
                        if (c == '\r' || c == '\n') {
                            if (m.cmd_len > 0) {
                                m.cmd_buf[m.cmd_len] = '\0';
                                handle_at(&m, m.cmd_buf);
                                m.cmd_len = 0;
                            }
                        } else if (c == 0x08 && m.cmd_len > 0) {
                            m.cmd_len--;
                        } else if (m.cmd_len < 254) {
                            m.cmd_buf[m.cmd_len++] = c;
                        }
                    }
                }
            }
        }

        /* Audio RX: read a block and feed to decoder */
        if (audio_idx >= 0 && (pfds[audio_idx].revents & POLLIN)) {
            ssize_t bytes = read(m.rx_fd, rx_audio, sizeof(rx_audio));
            if (bytes > 0) {
                size_t samples = (size_t)bytes / sizeof(float);
                if (m.state == MODEM_DATA)
                    psk_rx_feed(m.rx, rx_audio, samples);
                esc_audio_tick(&m, (int)samples);
            } else if (bytes == 0) {
                /* EOF on audio input */
                break;
            }
        }

        /* Audio TX: generate a block and write */
        if (m.tx) {
            psk_tx_read(m.tx, tx_audio, AUDIO_BLK);
            write(m.tx_fd, tx_audio, AUDIO_BLK * sizeof(float));
        }
    }

    log_msg("Modem exiting.");
    if (m.client_fd >= 0) close(m.client_fd);
    close(m.listen_fd);
    psk_tx_destroy(m.tx);
    psk_rx_destroy(m.rx);
    return 0;
}
