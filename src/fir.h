/*
 * fir.h — Biquad IIR and fixed-order FIR filter primitives.
 *
 * All state is in caller-owned structs; no dynamic allocation.
 * Inner loops use float throughout (ESP32-S3 hardware FPU compatible).
 *
 * MIT License. Copyright (c) 2025.
 */
#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Biquad IIR (direct form II transposed) ─────────────────────────────── */

/* Second-order section: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *                             - a1*y[n-1] - a2*y[n-2]             */
typedef struct {
    float b0, b1, b2; /* numerator coefficients */
    float a1, a2;     /* denominator coefficients (a0 normalised to 1) */
    float s1, s2;     /* filter state */
} psk_bq_t;

/* Initialise as 2nd-order bandpass (constant-peak-gain form).
 * f0: centre frequency (Hz), bw: -3 dB bandwidth (Hz), fs: sample rate (Hz). */
void psk_bq_init_bpf(psk_bq_t *bq, float f0, float bw, float fs);

/* Initialise as 2nd-order Butterworth lowpass.
 * fc: -3 dB cutoff (Hz), fs: sample rate (Hz). */
void psk_bq_init_lpf(psk_bq_t *bq, float fc, float fs);

/* Process one sample. */
static inline float psk_bq_step(psk_bq_t *bq, float x)
{
    float y = bq->b0 * x + bq->s1;
    bq->s1  = bq->b1 * x - bq->a1 * y + bq->s2;
    bq->s2  = bq->b2 * x - bq->a2 * y;
    return y;
}

/* Reset state. */
static inline void psk_bq_reset(psk_bq_t *bq)
{
    bq->s1 = bq->s2 = 0.0f;
}

/* ── Cascaded biquad ─────────────────────────────────────────────────────── */

#define PSK_BQ_CASCADE_MAX 4

typedef struct {
    psk_bq_t stages[PSK_BQ_CASCADE_MAX];
    int      n;
} psk_bqc_t;

/* n-stage Butterworth lowpass, cutoff fc Hz, sample rate fs Hz.
 * n must be 1..PSK_BQ_CASCADE_MAX. */
void psk_bqc_init_lpf(psk_bqc_t *c, int n, float fc, float fs);

static inline float psk_bqc_step(psk_bqc_t *c, float x)
{
    int i;
    for (i = 0; i < c->n; i++)
        x = psk_bq_step(&c->stages[i], x);
    return x;
}

static inline void psk_bqc_reset(psk_bqc_t *c)
{
    int i;
    for (i = 0; i < c->n; i++)
        psk_bq_reset(&c->stages[i]);
}

#ifdef __cplusplus
}
#endif
