/*
 * psk.h — Public API for libpsk: PSK31/63/125 modem library.
 *
 * MIT License. Copyright (c) 2025.
 * PSK31 algorithm by Peter Martinez G3PLX (public domain).
 *
 * Zero external dependencies. C99 API suitable for C, C++, Python (ctypes),
 * Rust (FFI). ESP32-S3 compatible (float only, no POSIX after init, <4 KB RAM
 * per context).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Mode ───────────────────────────────────────────────────────────────── */

typedef enum {
    PSK_MODE_BPSK31  = 0,
    PSK_MODE_QPSK31  = 1,
    PSK_MODE_BPSK63  = 2,
    PSK_MODE_BPSK125 = 3,
    PSK_MODE_QPSK63  = 4,
    PSK_MODE_QPSK125 = 5,
} psk_mode_t;

/* ── TX ─────────────────────────────────────────────────────────────────── */

typedef struct psk_tx_s psk_tx_t;

/* Create a TX context.
 * sample_rate: audio output sample rate in Hz (typically 8000 or 12000)
 * carrier_hz:  audio carrier frequency in Hz (typically 1000–2000)
 * mode:        PSK_MODE_*
 * Returns NULL on invalid parameters. */
psk_tx_t *psk_tx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode);
void      psk_tx_destroy(psk_tx_t *tx);

/* Write text to transmit; enqueues into internal ring buffer.
 * Returns number of characters accepted (may be less than len if buffer full). */
size_t psk_tx_write(psk_tx_t *tx, const char *text, size_t len);

/* Generate PCM samples.
 * out:    caller-allocated float buffer
 * frames: number of samples to generate
 * Returns number of samples written (always == frames unless tx is NULL).
 * Outputs idle carrier (amplitude-ramping to 0) when no text is queued. */
size_t psk_tx_read(psk_tx_t *tx, float *out, size_t frames);

/* True (non-zero) if internal buffer empty and tail idle transmitted. */
int psk_tx_idle(psk_tx_t const *tx);

/* ── RX ─────────────────────────────────────────────────────────────────── */

typedef struct psk_rx_s psk_rx_t;

/* Called for each decoded character. userdata is passed through unchanged. */
typedef void (*psk_rx_char_cb)(char c, void *userdata);

/* Called periodically with signal metrics. snr_db < -20 typically = noise. */
typedef void (*psk_rx_status_cb)(float snr_db, float freq_offset_hz, void *userdata);

psk_rx_t *psk_rx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode);
void      psk_rx_destroy(psk_rx_t *rx);

void psk_rx_set_char_callback  (psk_rx_t *rx, psk_rx_char_cb   cb, void *userdata);
void psk_rx_set_status_callback(psk_rx_t *rx, psk_rx_status_cb cb, void *userdata);

/* Feed PCM samples to the decoder. Callbacks invoked synchronously.
 * frames: number of float32 samples in buf */
void psk_rx_feed(psk_rx_t *rx, const float *buf, size_t frames);

/* Reset decoder state (e.g., after squelch open). */
void psk_rx_reset(psk_rx_t *rx);

/* ── Varicode utilities ─────────────────────────────────────────────────── */

/* Encode one ASCII character to its varicode bit sequence.
 * bits_out: caller buffer, at least 14 elements (max varicode + 2 stop bits).
 * Returns number of bits written (including 2 trailing zero separator bits).
 * Returns 0 for characters outside ASCII 0–127. */
int  psk_varicode_encode(char c, uint8_t *bits_out);

/* Decode accumulated bits to a character.
 * bits: array of 0/1 values, len: number of bits (NOT including trailing 00).
 * Returns ASCII character, or 0 if no match. */
char psk_varicode_decode(const uint8_t *bits, size_t len);

#ifdef __cplusplus
}
#endif
