/*
 * timing.c — Gardner symbol timing recovery.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "timing.h"
#include <math.h>

void psk_timing_init(psk_timing_t *t, int sps)
{
    t->sps         = sps;
    t->clock_inc   = 1.0f / (float)sps;
    t->clock_phase = 0.0f;
    t->K_timing    = 0.01f / (float)sps;   /* gentle loop: 0.01/sps */
    t->prev_I = t->prev_Q = 0.0f;
    t->mid_I  = t->mid_Q  = 0.0f;
    t->at_mid  = 0;
    t->half_clock = 0.5f;
}

void psk_timing_reset(psk_timing_t *t)
{
    t->clock_phase = 0.0f;
    t->prev_I = t->prev_Q = 0.0f;
    t->mid_I  = t->mid_Q  = 0.0f;
    t->at_mid  = 0;
}

int psk_timing_step(psk_timing_t *t, float I_in, float Q_in,
                    float *I_sym, float *Q_sym)
{
    int ret = 0;

    t->clock_phase += t->clock_inc;

    if (!t->at_mid && t->clock_phase >= t->half_clock) {
        /* Mid-symbol sample */
        t->mid_I  = I_in;
        t->mid_Q  = Q_in;
        t->at_mid = 1;
    }

    if (t->clock_phase >= 1.0f) {
        /* Symbol boundary — apply Gardner TED */
        float dI  = t->prev_I - I_in;
        float dQ  = t->prev_Q - Q_in;
        float err = dI * t->mid_I + dQ * t->mid_Q;

        /* Update clock (first-order loop).
         * Allow negative clock_phase so the next symbol can fire *later*
         * (delay correction); clamping to 0 was one-sided and prevented it. */
        t->clock_phase -= 1.0f;
        t->clock_phase += t->K_timing * err;
        if (t->clock_phase < -0.5f)  t->clock_phase = -0.5f;
        if (t->clock_phase >  0.5f)  t->clock_phase =  0.5f;

        /* Update previous sample */
        t->prev_I = I_in;
        t->prev_Q = Q_in;
        t->at_mid = 0;

        *I_sym = I_in;
        *Q_sym = Q_in;
        ret = 1;
    }

    return ret;
}
