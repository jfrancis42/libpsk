/*
 * costas.c — Second-order Costas loop implementation.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "costas.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*
 * Loop bandwidth Bn (normalised to symbol rate) relates to gains via:
 *   For a critically-damped second-order loop (ζ = 1/√2):
 *     Kp = 4 * ζ * Bn / (1 + 2*ζ*Bn + Bn²)
 *     Ki = 4 * Bn² / (1 + 2*ζ*Bn + Bn²)
 * We use Bn = bandwidth_hz / sample_rate (tiny fraction for PSK31).
 */
void psk_costas_init(psk_costas_t *c, float carrier_hz, float fs,
                     float bandwidth_hz, int qpsk)
{
    float Bn   = bandwidth_hz / fs;
    float zeta = 0.7071068f;   /* 1/sqrt(2), critical damping */
    float denom = 1.0f + 2.0f * zeta * Bn + Bn * Bn;

    c->nco_center  = 2.0f * (float)M_PI * carrier_hz / fs;
    c->nco_phase   = 0.0f;
    c->nco_freq    = 0.0f;
    c->integrator  = 0.0f;
    c->Kp          = 4.0f * zeta * Bn / denom;
    c->Ki          = 4.0f * Bn * Bn / denom;
    c->qpsk        = qpsk;
}

void psk_costas_reset(psk_costas_t *c)
{
    c->nco_phase  = 0.0f;
    c->nco_freq   = 0.0f;
    c->integrator = 0.0f;
}

void psk_costas_step(psk_costas_t *c, float x, float *I_out, float *Q_out)
{
    float phi, I, Q, err;

    /* Mix input with NCO */
    phi = c->nco_phase;
    I   =  x * cosf(phi);    /* I arm */
    Q   = -x * sinf(phi);    /* Q arm (note: −sin for correct BPSK convention) */

    *I_out = I;
    *Q_out = Q;

    /* Phase error detector */
    if (c->qpsk) {
        /* QPSK: four-quadrant error */
        float si = (I >= 0.0f) ? 1.0f : -1.0f;
        float sq = (Q >= 0.0f) ? 1.0f : -1.0f;
        err = si * Q - sq * I;
    } else {
        /* BPSK: standard Costas loop */
        float si = (I >= 0.0f) ? 1.0f : -1.0f;
        err = si * Q;
    }

    /* Loop filter (PI) */
    c->integrator += c->Ki * err;
    c->nco_freq   += c->integrator + c->Kp * err;

    /* Clamp frequency error to ±acquisition range */
    {
        float max_freq = 0.05f; /* radians/sample, ~64 Hz @ 8 kHz */
        if (c->nco_freq >  max_freq) c->nco_freq =  max_freq;
        if (c->nco_freq < -max_freq) c->nco_freq = -max_freq;
    }

    /* Advance NCO phase */
    c->nco_phase += c->nco_center + c->nco_freq;
    /* Wrap to [-π, π] */
    while (c->nco_phase >  (float)M_PI) c->nco_phase -= 2.0f * (float)M_PI;
    while (c->nco_phase < -(float)M_PI) c->nco_phase += 2.0f * (float)M_PI;
}
