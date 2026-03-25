/*
 * timing.h — Symbol timing recovery for PSK31.
 *
 * Uses the Gardner timing error detector (Gardner 1986), which computes:
 *   e(k) = Re(y[k-1] - y[k]) * Re(y[k - T/2])
 *         + Im(y[k-1] - y[k]) * Im(y[k - T/2])
 * where T = symbol period and indices are at symbol sampling instants.
 *
 * A first-order control loop adjusts the fractional sample clock phase.
 * The caller feeds every sample; the module signals when to sample a symbol.
 *
 * MIT License. Copyright (c) 2025.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float  clock_phase;     /* current clock phase, range [0, 1) */
    float  clock_inc;       /* nominal clock increment per sample = 1/sps */
    float  K_timing;        /* timing loop gain */

    /* Sample buffer at half-symbol rate for Gardner detector */
    float  prev_I, prev_Q;        /* previous symbol sample (t_k-1) */
    float  mid_I,  mid_Q;         /* mid-symbol sample (t_k - T/2) */

    int    sps;             /* samples per symbol */
    int    at_mid;          /* flag: next trigger is mid-symbol sample */
    float  half_clock;      /* 0.5 threshold for mid sample */
} psk_timing_t;

/* Initialise timing recovery.
 * sps: samples per symbol (e.g. 256 for BPSK31 at 8 kHz) */
void psk_timing_init(psk_timing_t *t, int sps);

/* Reset loop state. */
void psk_timing_reset(psk_timing_t *t);

/*
 * Feed one I/Q sample pair.
 * Returns:  0 — no symbol yet
 *           1 — symbol sampling instant: *I_sym, *Q_sym updated
 *
 * The timing error detector runs only at symbol instants.
 */
int psk_timing_step(psk_timing_t *t, float I_in, float Q_in,
                    float *I_sym, float *Q_sym);

#ifdef __cplusplus
}
#endif
