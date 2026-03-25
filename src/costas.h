/*
 * costas.h — Second-order Costas loop for BPSK/QPSK carrier recovery.
 *
 * The Costas loop drives a phase/frequency error toward zero using:
 *   BPSK error: e = sign(I) * Q
 *   QPSK error: e = sign(I) * Q - sign(Q) * I   (4-quadrant)
 *
 * Loop filter is a PI controller:
 *   integrator += Ki * e
 *   nco_freq   += integrator + Kp * e
 *
 * The NCO generates I = cos(φ) and Q = -sin(φ) mixing tones that are
 * multiplied by the input to produce baseband I and Q.
 *
 * MIT License. Copyright (c) 2025.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* NCO */
    float nco_phase;     /* current NCO phase (radians) */
    float nco_freq;      /* NCO frequency offset (radians/sample) */
    float nco_center;    /* center frequency (radians/sample = 2π*f/fs) */

    /* Loop filter PI state */
    float integrator;

    /* Loop gains (set by psk_costas_init) */
    float Kp;
    float Ki;

    /* QPSK flag */
    int   qpsk;
} psk_costas_t;

/* Initialise Costas loop.
 * carrier_hz:    nominal carrier frequency (Hz)
 * fs:            sample rate (Hz)
 * bandwidth_hz:  one-sided acquisition bandwidth (Hz); typical = 2×baud_rate
 * qpsk:          0 = BPSK error detector, 1 = QPSK error detector */
void psk_costas_init(psk_costas_t *c, float carrier_hz, float fs,
                     float bandwidth_hz, int qpsk);

/* Reset loop state (keep gains, clear phase/integrator). */
void psk_costas_reset(psk_costas_t *c);

/* Process one input sample.
 * Inputs:  x — bandpass-filtered PCM sample
 * Outputs: *I, *Q — baseband I and Q after mixing and error correction.
 * Call once per sample. */
void psk_costas_step(psk_costas_t *c, float x, float *I, float *Q);

#ifdef __cplusplus
}
#endif
