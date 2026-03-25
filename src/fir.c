/*
 * fir.c — Biquad IIR filter coefficient computation.
 *
 * Uses the bilinear transform (cookbook formulas from Audio EQ Cookbook,
 * R. Bristow-Johnson, public domain).
 *
 * MIT License. Copyright (c) 2025.
 */

#include "fir.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Biquad BPF (constant-peak-gain form) ─────────────────────────────── */

void psk_bq_init_bpf(psk_bq_t *bq, float f0, float bw, float fs)
{
    float w0    = 2.0f * (float)M_PI * f0 / fs;
    /* alpha = sin(w0)/(2*Q) where Q = f0/bw (Q-factor formula).
     * Previous formula used bw/fs instead of bw/(2*f0), producing an
     * effective bandwidth ~10× narrower than intended (causing ISI). */
    float alpha = sinf(w0) * bw / (2.0f * f0);
    float a0    = 1.0f + alpha;

    bq->b0 =  alpha        / a0;
    bq->b1 =  0.0f;
    bq->b2 = -alpha        / a0;
    bq->a1 = -2.0f * cosf(w0) / a0;
    bq->a2 = (1.0f - alpha) / a0;
    bq->s1 = bq->s2 = 0.0f;
}

/* ── Biquad LPF (2nd-order Butterworth) ────────────────────────────────── */

void psk_bq_init_lpf(psk_bq_t *bq, float fc, float fs)
{
    float w0    = 2.0f * (float)M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * 0.7071068f); /* Q = 1/sqrt(2) */
    float a0    = 1.0f + alpha;

    bq->b0 = (1.0f - cosf(w0)) / 2.0f / a0;
    bq->b1 =  (1.0f - cosf(w0))        / a0;
    bq->b2 = (1.0f - cosf(w0)) / 2.0f / a0;
    bq->a1 = -2.0f * cosf(w0)          / a0;
    bq->a2 = (1.0f - alpha)            / a0;
    bq->s1 = bq->s2 = 0.0f;
}

/* ── Cascaded Butterworth LPF ─────────────────────────────────────────── */
/*
 * An n-th order Butterworth LPF is implemented as n/2 (for even n) biquad
 * stages with different Q factors.  For the standard Butterworth pole angles:
 *   Q_k = 1 / (2 * cos(π*(2k+1)/(2n)))   for k = 0..n/2-1
 */
void psk_bqc_init_lpf(psk_bqc_t *c, int n, float fc, float fs)
{
    int k;

    if (n < 1) n = 1;
    if (n > PSK_BQ_CASCADE_MAX * 2) n = PSK_BQ_CASCADE_MAX * 2;
    c->n = (n + 1) / 2; /* number of biquad stages (ceil(n/2)) */

    for (k = 0; k < c->n; k++) {
        psk_bq_t *bq = &c->stages[k];
        float total_order = (float)n;
        /* Butterworth pole angle for this stage */
        float theta = (float)M_PI * (2.0f * (float)k + 1.0f) / (2.0f * total_order);
        float Q     = 1.0f / (2.0f * cosf(theta));
        float w0    = 2.0f * (float)M_PI * fc / fs;
        float alpha = sinf(w0) / (2.0f * Q);
        float a0    = 1.0f + alpha;

        bq->b0 = (1.0f - cosf(w0)) / 2.0f / a0;
        bq->b1 =  (1.0f - cosf(w0))        / a0;
        bq->b2 = (1.0f - cosf(w0)) / 2.0f / a0;
        bq->a1 = -2.0f * cosf(w0)          / a0;
        bq->a2 = (1.0f - alpha)            / a0;
        bq->s1 = bq->s2 = 0.0f;
    }

    /* If n is odd, the last stage is a first-order section implemented as
     * a degenerate biquad (b2=a2=0). */
    if (n & 1) {
        psk_bq_t *bq = &c->stages[c->n - 1];
        float wc  = tanf((float)M_PI * fc / fs);
        float a0  = 1.0f + wc;
        bq->b0 = wc / a0;
        bq->b1 = wc / a0;
        bq->b2 = 0.0f;
        bq->a1 = (wc - 1.0f) / a0;
        bq->a2 = 0.0f;
        bq->s1 = bq->s2 = 0.0f;
    }
}
