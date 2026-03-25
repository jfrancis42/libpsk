/*
 * rx.c — PSK31/63/125 receiver.
 *
 * Signal chain:
 *   PCM float32
 *     → Biquad BPF (pre-filter, ±3× baud_rate)
 *     → I/Q downconversion (mix with NCO cos/sin)
 *     → 4-pole Butterworth LPF on I and Q (cutoff = 1.5× baud_rate)
 *     → Costas loop error computed from LPF'd I/Q → update NCO
 *     → Symbol timing recovery (Gardner TED on filtered I/Q)
 *     → Phase detector (atan2(Q, I) at symbol instants)
 *     → Differential decoder
 *     → (QPSK: Viterbi decoder)
 *     → Varicode shift register → char callback
 *
 * Key design note: the Costas loop error is computed AFTER the LPF so that
 * double-frequency mixer products don't corrupt the loop filter.  The loop
 * updates every sample but uses the clean (LPF'd) I/Q.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "psk.h"
#include "fir.h"
#include "costas.h"
#include "timing.h"
#include "viterbi.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Mode parameters ─────────────────────────────────────────────────────── */

typedef struct { float baud; int qpsk; } rx_mode_parms_t;

static const rx_mode_parms_t RX_MODE[6] = {
    { 31.25f, 0 },   /* BPSK31  */
    { 31.25f, 1 },   /* QPSK31  */
    { 62.5f,  0 },   /* BPSK63  */
    { 125.0f, 0 },   /* BPSK125 */
    { 62.5f,  1 },   /* QPSK63  */
    { 125.0f, 1 },   /* QPSK125 */
};

/* ── Varicode accumulator ─────────────────────────────────────────────────── */

/*
 * The varicode stream is: [codeword][00][codeword][00]...
 * Codewords start with 1 and contain no internal "00" sequence.
 * Strategy: buffer each bit; emit a character when two consecutive 0-bits
 * are seen.  Use a one-bit lookahead so we know whether a 0-bit is an
 * interior codeword zero (followed by 1) or the start of the separator
 * (followed by another 0).
 */
typedef struct {
    uint16_t accum;        /* accumulated codeword bits (MSB first) */
    int      len;          /* number of bits in accum */
    int      active;       /* 1 once the first 1-bit of a codeword is seen */
    int      pending_zero; /* 1 if the previous bit was a 0 not yet committed */
} vacc_t;

static void vacc_reset(vacc_t *v)
{
    v->accum        = 0;
    v->len          = 0;
    v->active       = 0;
    v->pending_zero = 0;
}

/*
 * Push one decoded bit.  Returns ASCII char when 00 separator detected,
 * 0 otherwise.
 * bit: 1 = no phase change, 0 = phase change (BPSK differential convention).
 */
static char vacc_push(vacc_t *v, int bit)
{
    if (bit) {
        /* 1-bit: commit any pending zero then push this 1 */
        if (v->pending_zero) {
            v->accum = (uint16_t)(v->accum << 1); /* push 0 */
            v->len++;
            v->pending_zero = 0;
        }
        v->active = 1;
        v->accum  = (uint16_t)((v->accum << 1) | 1);
        v->len++;
        if (v->len > 14)
            vacc_reset(v); /* overrun protection */
    } else {
        /* 0-bit */
        if (!v->active) {
            /* Not in a codeword yet (e.g. preamble) — ignore */
            return 0;
        }
        if (v->pending_zero) {
            /* Two consecutive zeros = inter-character separator */
            char    c = 0;
            if (v->len > 0 && v->len <= 14) {
                uint8_t bits[16];
                int     i;
                for (i = v->len - 1; i >= 0; i--)
                    bits[v->len - 1 - i] = (v->accum >> i) & 1;
                c = psk_varicode_decode(bits, (size_t)v->len);
            }
            vacc_reset(v);
            return c;
        }
        /* First of a potential separator pair — hold it */
        v->pending_zero = 1;
    }
    return 0;
}

/* ── RX struct ─────────────────────────────────────────────────────────────── */

struct psk_rx_s {
    /* Mode */
    psk_mode_t mode;
    unsigned   sample_rate;
    float      carrier_hz;
    float      baud;
    int        is_qpsk;

    /* Callbacks */
    psk_rx_char_cb   char_cb;
    void            *char_ud;
    psk_rx_status_cb status_cb;
    void            *status_ud;

    /* Pre-filter BPF */
    psk_bq_t bpf;

    /* LPF on I/Q arms after downconversion */
    psk_bqc_t lpf_I;
    psk_bqc_t lpf_Q;

    /* Costas loop state (inlined for correct LPF-after-mix ordering) */
    float nco_phase;
    float nco_center;   /* 2π * carrier_hz / sample_rate */
    float nco_freq;     /* frequency correction (rad/sample) */
    float costas_int;   /* integrator */
    float costas_Kp;
    float costas_Ki;

    /* Symbol timing recovery */
    psk_timing_t timing;

    /* Differential decoder */
    float prev_phase;
    int   got_first;

    /* Varicode accumulator */
    vacc_t vacc;

    /* QPSK Viterbi decoder */
    psk_viterbi_t viterbi;

    /* SNR / power estimator */
    float sig_power;
    float noise_power;
    float snr_alpha;

    /* Status callback throttle */
    unsigned status_count;
    unsigned status_interval;
};

/* ── Public API ──────────────────────────────────────────────────────────── */

psk_rx_t *psk_rx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode)
{
    psk_rx_t *rx;
    float baud, bpf_bw, lpf_fc, loop_bw;
    float Bn, zeta, denom;

    if (sample_rate == 0 || carrier_hz <= 0.0f || carrier_hz >= sample_rate / 2.0f)
        return NULL;
    if ((int)mode < 0 || (int)mode > 5)
        return NULL;

    rx = (psk_rx_t *)calloc(1, sizeof(*rx));
    if (!rx) return NULL;

    rx->mode        = mode;
    rx->sample_rate = sample_rate;
    rx->carrier_hz  = carrier_hz;
    rx->baud        = RX_MODE[mode].baud;
    rx->is_qpsk     = RX_MODE[mode].qpsk;

    baud    = rx->baud;
    bpf_bw  = baud * 8.0f;     /* pre-filter BW: 8× baud (~22 Hz effective at BPSK31) */
    lpf_fc  = baud * 1.5f;     /* baseband LPF: 1.5× baud — tight for noise rejection */
    loop_bw = baud * 0.05f;    /* Costas loop BW: 0.05× baud */

    /* Pre-filter BPF */
    psk_bq_init_bpf(&rx->bpf, carrier_hz, bpf_bw, (float)sample_rate);

    /* Baseband I/Q LPF — 1st-order at 1.5×baud.
     * Tight enough to suppress noise outside the signal band (~1.5× baud)
     * without significant ISI; group delay near-zero (< 1 sample at DC). */
    psk_bqc_init_lpf(&rx->lpf_I, 1, lpf_fc, (float)sample_rate);
    psk_bqc_init_lpf(&rx->lpf_Q, 1, lpf_fc, (float)sample_rate);

    /* Costas loop gains (critically damped, ζ = 1/√2) */
    rx->nco_center = 2.0f * (float)M_PI * carrier_hz / (float)sample_rate;
    rx->nco_phase  = 0.0f;
    rx->nco_freq   = 0.0f;
    rx->costas_int = 0.0f;
    Bn    = loop_bw / (float)sample_rate;
    zeta  = 0.7071068f;
    denom = 1.0f + 2.0f * zeta * Bn + Bn * Bn;
    rx->costas_Kp = 4.0f * zeta * Bn / denom;
    rx->costas_Ki = 4.0f * Bn * Bn / denom;

    /* Symbol timing */
    psk_timing_init(&rx->timing, (int)((float)sample_rate / baud + 0.5f));

    /* Power estimator: time constant ~0.1 s */
    rx->snr_alpha      = 1.0f - expf(-1.0f / ((float)sample_rate * 0.1f));
    rx->status_interval = (unsigned)((float)sample_rate / baud);

    /* Viterbi (for QPSK modes) */
    psk_viterbi_reset(&rx->viterbi);

    vacc_reset(&rx->vacc);

    return rx;
}

void psk_rx_destroy(psk_rx_t *rx)
{
    free(rx);
}

void psk_rx_set_char_callback(psk_rx_t *rx, psk_rx_char_cb cb, void *userdata)
{
    if (rx) { rx->char_cb = cb; rx->char_ud = userdata; }
}

void psk_rx_set_status_callback(psk_rx_t *rx, psk_rx_status_cb cb, void *userdata)
{
    if (rx) { rx->status_cb = cb; rx->status_ud = userdata; }
}

void psk_rx_reset(psk_rx_t *rx)
{
    if (!rx) return;
    psk_bq_reset(&rx->bpf);
    psk_bqc_reset(&rx->lpf_I);
    psk_bqc_reset(&rx->lpf_Q);
    rx->nco_phase  = 0.0f;
    rx->nco_freq   = 0.0f;
    rx->costas_int = 0.0f;
    psk_timing_reset(&rx->timing);
    psk_viterbi_reset(&rx->viterbi);
    vacc_reset(&rx->vacc);
    rx->prev_phase   = 0.0f;
    rx->got_first    = 0;
    rx->sig_power    = 0.0f;
    rx->noise_power  = 0.0f;
    rx->status_count = 0;
}

void psk_rx_feed(psk_rx_t *rx, const float *buf, size_t frames)
{
    size_t i;
    if (!rx || !buf) return;

    for (i = 0; i < frames; i++) {
        float x  = buf[i];
        float bp, I_raw, Q_raw, I_filt, Q_filt, I_sym, Q_sym;
        float err;

        /* Power estimators */
        rx->noise_power = rx->noise_power * (1.0f - rx->snr_alpha)
                        + x * x * rx->snr_alpha;

        /* Pre-filter BPF */
        bp = psk_bq_step(&rx->bpf, x);

        rx->sig_power = rx->sig_power * (1.0f - rx->snr_alpha)
                      + bp * bp * rx->snr_alpha;

        /* ── Downconvert with NCO ──────────────────────────────────────── */
        {
            float phi = rx->nco_phase;
            I_raw =  bp * cosf(phi);
            Q_raw = -bp * sinf(phi);
        }

        /* ── LPF ──────────────────────────────────────────────────────── */
        I_filt = psk_bqc_step(&rx->lpf_I, I_raw);
        Q_filt = psk_bqc_step(&rx->lpf_Q, Q_raw);

        /* ── Costas loop error from filtered I/Q ──────────────────────── */
        if (rx->is_qpsk) {
            float si = (I_filt >= 0.0f) ? 1.0f : -1.0f;
            float sq = (Q_filt >= 0.0f) ? 1.0f : -1.0f;
            err = si * Q_filt - sq * I_filt;
        } else {
            float si = (I_filt >= 0.0f) ? 1.0f : -1.0f;
            err = si * Q_filt;
        }

        /* ── Loop filter update ───────────────────────────────────────── */
        rx->costas_int += rx->costas_Ki * err;
        rx->nco_freq    = rx->costas_int + rx->costas_Kp * err;
        {
            float max_f = 0.03f;  /* ±38 Hz @ 8 kHz */
            if (rx->nco_freq >  max_f) rx->nco_freq =  max_f;
            if (rx->nco_freq < -max_f) rx->nco_freq = -max_f;
        }

        /* ── Advance NCO ─────────────────────────────────────────────── */
        rx->nco_phase += rx->nco_center + rx->nco_freq;
        while (rx->nco_phase >  (float)M_PI) rx->nco_phase -= 2.0f * (float)M_PI;
        while (rx->nco_phase < -(float)M_PI) rx->nco_phase += 2.0f * (float)M_PI;

        /* ── Symbol timing recovery ───────────────────────────────────── */
        if (!psk_timing_step(&rx->timing, I_filt, Q_filt, &I_sym, &Q_sym))
            goto next_sample;

        /* ── Symbol decision ─────────────────────────────────────────── */
        {
            float phase = atan2f(Q_sym, I_sym);

            if (!rx->got_first) {
                rx->prev_phase = phase;
                rx->got_first  = 1;
                goto next_sample;
            }

            if (rx->is_qpsk) {
                /* QPSK differential: phase difference → dibit */
                float dp = phase - rx->prev_phase;
                while (dp >  (float)M_PI) dp -= 2.0f * (float)M_PI;
                while (dp < -(float)M_PI) dp += 2.0f * (float)M_PI;

                /* Quantise to nearest 90° → dibit 0..3 */
                int dibit;
                {
                    float adp = dp;
                    if (adp < 0.0f) adp += 2.0f * (float)M_PI;
                    dibit = (int)(adp / ((float)M_PI / 2.0f) + 0.5f) & 3;
                }
                {
                    float soft0 = (float)(dibit >> 1) * 2.0f - 1.0f;
                    float soft1 = (float)(dibit &  1) * 2.0f - 1.0f;
                    int dbit = psk_viterbi_decode(&rx->viterbi, soft0, soft1);
                    if (dbit >= 0) {
                        char c = vacc_push(&rx->vacc, dbit);
                        if (c && rx->char_cb)
                            rx->char_cb(c, rx->char_ud);
                    }
                }
            } else {
                /* BPSK differential: |dp| > π/2 → phase change → bit=0 */
                float dp = phase - rx->prev_phase;
                while (dp >  (float)M_PI) dp -= 2.0f * (float)M_PI;
                while (dp < -(float)M_PI) dp += 2.0f * (float)M_PI;

                int bit = (fabsf(dp) < (float)M_PI / 2.0f) ? 1 : 0;
                {
                    char c = vacc_push(&rx->vacc, bit);
                    if (c && rx->char_cb)
                        rx->char_cb(c, rx->char_ud);
                }
            }

            rx->prev_phase = phase;
        }

        /* ── Status callback ─────────────────────────────────────────── */
        rx->status_count++;
        if (rx->status_count >= rx->status_interval) {
            rx->status_count = 0;
            if (rx->status_cb) {
                float snr_db  = -99.0f;
                float freq_off = rx->nco_freq * (float)rx->sample_rate
                               / (2.0f * (float)M_PI);
                if (rx->sig_power > 1e-12f && rx->noise_power > rx->sig_power) {
                    float noise_only = rx->noise_power - rx->sig_power;
                    snr_db = 10.0f * log10f(rx->sig_power / noise_only);
                }
                rx->status_cb(snr_db, freq_off, rx->status_ud);
            }
        }

    next_sample:;
    }
}
