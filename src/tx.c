/*
 * tx.c — PSK31/63/125 transmitter.
 *
 * Signal chain:
 *   Text char → Varicode bits → (QPSK: conv. encoder) → symbols
 *               → raised-cosine amplitude shaping → PCM float32
 *
 * BPSK differential encoding:
 *   bit 1 → no phase change (keep current phase)
 *   bit 0 → phase inversion (add π)
 *
 * Amplitude shaping at phase-change boundaries (raised half-cosine):
 *   No phase change:  amplitude = 1.0 throughout symbol
 *   Phase change:     amplitude = |cos(π * n / N)|, n = 0..N-1
 *                     The sign of cos() naturally implements the phase flip
 *                     at the midpoint (cos goes negative after N/2).
 *
 * ESP32-S3 compatible: no dynamic allocation after psk_tx_create(), float only.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "psk.h"
#include "viterbi.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Mode parameters ────────────────────────────────────────────────────── */

typedef struct {
    float baud;    /* symbol rate */
    int   qpsk;    /* 0=BPSK, 1=QPSK */
} mode_parms_t;

static const mode_parms_t MODE_TABLE[6] = {
    { 31.25f, 0 },  /* BPSK31  */
    { 31.25f, 1 },  /* QPSK31  */
    { 62.5f,  0 },  /* BPSK63  */
    { 125.0f, 0 },  /* BPSK125 */
    { 62.5f,  1 },  /* QPSK63  */
    { 125.0f, 1 },  /* QPSK125 */
};

/* ── TX struct ──────────────────────────────────────────────────────────── */

#define TX_TEXT_BUF  256      /* characters */
#define TX_BIT_BUF   32       /* bits in staging queue */
#define PREAMBLE_SYMS 32      /* idle symbols before/after data */

/* TX state machine states */
typedef enum {
    TX_STATE_IDLE,      /* no text, generating silence */
    TX_STATE_PREAMBLE,  /* sending preamble (all-zero bits = continuous phase changes) */
    TX_STATE_DATA,      /* sending varicode bits */
    TX_STATE_POSTAMBLE, /* flush trailing idle symbols */
} tx_state_t;

struct psk_tx_s {
    /* Mode/parameters */
    psk_mode_t mode;
    unsigned   sample_rate;
    float      carrier_hz;
    int        sps;           /* samples per symbol */
    float      baud;
    int        is_qpsk;

    /* Carrier NCO */
    float carrier_phase;
    float carrier_inc;        /* 2π * carrier_hz / sample_rate */

    /* Per-symbol state */
    float cur_amp_sign;       /* +1 or -1 (tracks phase polarity) */
    int   phase_change;       /* is current symbol a phase change? */
    int   sample_in_sym;      /* 0 .. sps-1 */
    float inv_sps;            /* 1/sps precomputed */

    /* Bit staging FIFO (holds varicode bits before shaping) */
    uint8_t bits[TX_BIT_BUF];
    int     bit_r, bit_w;

    /* Text ring buffer */
    char    text[TX_TEXT_BUF];
    int     text_r, text_w;

    /* State machine */
    tx_state_t state;
    int        preamble_left;   /* symbols left in preamble/postamble */

    /* QPSK convolutional encoder */
    psk_conv_t conv;

    /* QPSK interleaver (depth 8 dibits = 16 bits) */
    uint8_t il_buf[16];
    int     il_in, il_out, il_fill;

    /* QPSK differential encoder */
    int     diff_phase;   /* current phase index 0..3 */
};

/* ── Helpers ────────────────────────────────────────────────────────────── */

static inline int bits_avail(const psk_tx_t *tx)
{
    return (tx->bit_w - tx->bit_r + TX_BIT_BUF) % TX_BIT_BUF;
}

static inline int text_avail(const psk_tx_t *tx)
{
    return (tx->text_w - tx->text_r + TX_TEXT_BUF) % TX_TEXT_BUF;
}

static inline void push_bit(psk_tx_t *tx, int b)
{
    tx->bits[tx->bit_w % TX_BIT_BUF] = (uint8_t)(b & 1);
    tx->bit_w++;
    if (tx->bit_w - tx->bit_r > TX_BIT_BUF)
        tx->bit_r++; /* overflow: drop oldest — should not happen in normal use */
}

static inline int pop_bit(psk_tx_t *tx)
{
    if (tx->bit_w == tx->bit_r) return -1;
    return tx->bits[tx->bit_r++ % TX_BIT_BUF];
}

/* Push one character's varicode bits + 00 separator into the bit queue. */
static void push_char(psk_tx_t *tx, char c)
{
    uint8_t vbits[16];
    int     n = psk_varicode_encode(c, vbits);
    int     i;
    if (tx->is_qpsk) {
        /* QPSK: run bits through convolutional encoder before queuing.
         * Each input bit → 2 output bits (held in interleaver). */
        for (i = 0; i < n; i++) {
            int o0, o1;
            psk_conv_encode(&tx->conv, vbits[i], &o0, &o1);
            /* Simple block interleaver: buffer 8 symbols (16 bits), then
             * output in column-interleaved order. */
            tx->il_buf[tx->il_in & 15] = (uint8_t)o0;
            tx->il_in++;
            tx->il_buf[tx->il_in & 15] = (uint8_t)o1;
            tx->il_in++;
            tx->il_fill += 2;
            /* When interleaver has ≥16 bits, flush 2 bits (one dibit) */
            while (tx->il_fill >= 16) {
                push_bit(tx, tx->il_buf[(tx->il_out + 0) & 15]);
                push_bit(tx, tx->il_buf[(tx->il_out + 8) & 15]);
                tx->il_out += 2;
                tx->il_fill -= 2;
            }
        }
    } else {
        for (i = 0; i < n; i++)
            push_bit(tx, vbits[i]);
    }
}

/* Refill bit queue from text buffer until bit queue has ≥14 bits or empty. */
static void refill_bits(psk_tx_t *tx)
{
    while (bits_avail(tx) < TX_BIT_BUF - 14 && text_avail(tx) > 0) {
        char c = tx->text[tx->text_r++ % TX_TEXT_BUF];
        push_char(tx, c);
    }
}

/* QPSK: encode a dibit (bits b0, b1) into a QPSK phase index (0..3).
 * Uses differential encoding: output = (previous_phase + delta) mod 4.
 * Mapping: dibit 00→0°, 01→90°, 10→180°, 11→270°. */
static int qpsk_diff_encode(psk_tx_t *tx, int b0, int b1)
{
    int dibit  = (b0 << 1) | b1;
    tx->diff_phase = (tx->diff_phase + dibit) & 3;
    return tx->diff_phase;
}

/* Get next symbol's phase change (radians) for BPSK or QPSK.
 * Returns 0.0 (no change) or π (BPSK) or 0/π/2/3π/2 (QPSK) delta from current. */
static float next_symbol_phase_delta(psk_tx_t *tx)
{
    static const float qpsk_phases[4] = {
        0.0f,
        (float)M_PI / 2.0f,
        (float)M_PI,
        3.0f * (float)M_PI / 2.0f
    };

    if (tx->is_qpsk) {
        int b0 = pop_bit(tx);
        int b1 = pop_bit(tx);
        if (b0 < 0 || b1 < 0) return 0.0f;  /* idle */
        int idx = qpsk_diff_encode(tx, b0, b1);
        /* Return absolute phase (not delta) — we track diff_phase */
        return qpsk_phases[idx];
    } else {
        int b = pop_bit(tx);
        if (b < 0) return 0.0f;  /* idle */
        return (b == 0) ? (float)M_PI : 0.0f;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

psk_tx_t *psk_tx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode)
{
    psk_tx_t *tx;

    if (sample_rate == 0 || carrier_hz <= 0.0f || carrier_hz >= sample_rate / 2.0f)
        return NULL;
    if ((int)mode < 0 || (int)mode > 5)
        return NULL;

    tx = (psk_tx_t *)calloc(1, sizeof(*tx));
    if (!tx) return NULL;

    tx->mode        = mode;
    tx->sample_rate = sample_rate;
    tx->carrier_hz  = carrier_hz;
    tx->baud        = MODE_TABLE[mode].baud;
    tx->is_qpsk     = MODE_TABLE[mode].qpsk;
    tx->sps         = (int)(sample_rate / tx->baud + 0.5f);
    tx->inv_sps     = 1.0f / (float)tx->sps;
    tx->carrier_inc = 2.0f * (float)M_PI * carrier_hz / (float)sample_rate;
    tx->carrier_phase = 0.0f;
    tx->cur_amp_sign  = 1.0f;
    tx->phase_change  = 0;
    tx->sample_in_sym = 0;
    tx->state         = TX_STATE_IDLE;

    psk_conv_reset(&tx->conv);

    return tx;
}

void psk_tx_destroy(psk_tx_t *tx)
{
    free(tx);
}

size_t psk_tx_write(psk_tx_t *tx, const char *text, size_t len)
{
    size_t i, accepted = 0;
    int space;

    if (!tx || !text) return 0;

    for (i = 0; i < len; i++) {
        space = (TX_TEXT_BUF - 1) - text_avail(tx);
        if (space <= 0) break;
        tx->text[tx->text_w++ % TX_TEXT_BUF] = text[i];
        accepted++;
    }

    /* Kick state machine out of idle */
    if (tx->state == TX_STATE_IDLE && text_avail(tx) > 0) {
        tx->state         = TX_STATE_PREAMBLE;
        tx->preamble_left = PREAMBLE_SYMS;
        tx->phase_change  = 1; /* start preamble immediately with phase changes */
    }

    return accepted;
}

int psk_tx_idle(psk_tx_t const *tx)
{
    if (!tx) return 1;
    return (tx->state == TX_STATE_IDLE);
}

size_t psk_tx_read(psk_tx_t *tx, float *out, size_t frames)
{
    size_t n;
    if (!tx || !out) return 0;

    for (n = 0; n < frames; n++) {
        float amp, sample;

        /* -- Compute amplitude for current sample ------------------------- */

        if (tx->state == TX_STATE_IDLE) {
            out[n] = 0.0f;
            continue;
        }

        if (tx->phase_change) {
            /* Raised half-cosine envelope centred at symbol midpoint.
             *   amp(k) = cur_amp_sign * cos(π * k / N)
             * This ensures amplitude CONTINUITY across symbol boundaries:
             *   - Starts at cur_amp_sign (matching end of previous symbol)
             *   - Crosses zero at the midpoint (the actual phase transition)
             *   - Ends at -cur_amp_sign (which cur_amp_sign will flip to next) */
            float theta = (float)M_PI * (float)tx->sample_in_sym * tx->inv_sps;
            amp = tx->cur_amp_sign * cosf(theta);
        } else {
            amp = tx->cur_amp_sign;
        }

        sample = amp * cosf(tx->carrier_phase);
        out[n] = sample;

        /* Advance carrier */
        tx->carrier_phase += tx->carrier_inc;
        while (tx->carrier_phase > (float)M_PI)
            tx->carrier_phase -= 2.0f * (float)M_PI;

        tx->sample_in_sym++;

        /* -- End of symbol? ----------------------------------------------- */
        if (tx->sample_in_sym >= tx->sps) {
            tx->sample_in_sym = 0;

            /* Update amp_sign: if phase change occurred, sign flipped */
            if (tx->phase_change)
                tx->cur_amp_sign = -tx->cur_amp_sign;

            /* Get next symbol */
            switch (tx->state) {
            case TX_STATE_PREAMBLE:
                /* Preamble: all zeros (continuous phase changes) */
                tx->phase_change = 1;
                tx->preamble_left--;
                if (tx->preamble_left <= 0) {
                    tx->state = TX_STATE_DATA;
                    /* Pop the first data bit immediately so there is no
                     * spurious no-change symbol at the preamble/data boundary. */
                    refill_bits(tx);
                    if (bits_avail(tx) > 0) {
                        float delta = next_symbol_phase_delta(tx);
                        tx->phase_change = (delta != 0.0f) ? 1 : 0;
                    } else if (text_avail(tx) == 0) {
                        tx->state         = TX_STATE_POSTAMBLE;
                        tx->preamble_left = PREAMBLE_SYMS;
                        tx->phase_change  = 1;
                    } else {
                        tx->phase_change = 0; /* stall */
                    }
                }
                break;

            case TX_STATE_DATA:
                refill_bits(tx);
                if (bits_avail(tx) > 0) {
                    float delta;
                    if (tx->is_qpsk) {
                        /* For QPSK, phase_change based on absolute phase */
                        /* next_symbol_phase_delta returns the NEW absolute phase */
                        float new_abs = next_symbol_phase_delta(tx);
                        /* We track phase as cur_amp_sign = ±1 (BPSK style).
                         * For QPSK we use a separate approach. */
                        /* TODO: full QPSK I/Q modulation — simplified here */
                        tx->phase_change = (new_abs != 0.0f) ? 1 : 0;
                    } else {
                        delta = next_symbol_phase_delta(tx);
                        tx->phase_change = (delta != 0.0f) ? 1 : 0;
                    }
                } else if (text_avail(tx) == 0) {
                    /* No more text; start postamble */
                    tx->state         = TX_STATE_POSTAMBLE;
                    tx->preamble_left = PREAMBLE_SYMS;
                    tx->phase_change  = 1;
                } else {
                    tx->phase_change = 0; /* stall with idle carrier */
                }
                break;

            case TX_STATE_POSTAMBLE:
                tx->phase_change = 1;
                tx->preamble_left--;
                if (tx->preamble_left <= 0) {
                    tx->state        = TX_STATE_IDLE;
                    tx->phase_change = 0;
                }
                break;

            default:
                tx->phase_change = 0;
                break;
            }
        }
    }

    return frames;
}
