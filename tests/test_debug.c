/*
 * test_debug.c — Debug demodulation using same IIR LPF + timing as rx.c.
 * Replicates the exact rx.c signal chain but prints per-symbol values.
 */
#include "psk.h"
#include "../src/fir.h"
#include "../src/timing.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    const unsigned FS   = 8000;
    const float    FREQ = 1000.0f;
    const unsigned NSAMP = FS * 5;
    const float    baud  = 31.25f;
    const int      SPS   = 256;

    float *pcm = (float *)malloc(NSAMP * sizeof(float));
    if (!pcm) return 1;

    /* Generate TX audio */
    psk_tx_t *tx = psk_tx_create(FS, FREQ, PSK_MODE_BPSK31);
    psk_tx_write(tx, "HELLO PSK", 9);
    psk_tx_read(tx, pcm, NSAMP);
    psk_tx_destroy(tx);

    /* Build the same signal chain as rx.c */
    psk_bq_t  bpf;
    psk_bqc_t lpf_I, lpf_Q;
    psk_timing_t timing;

    psk_bq_init_bpf(&bpf, FREQ, baud * 8.0f, (float)FS);
    psk_bqc_init_lpf(&lpf_I, 1, baud * 4.0f, (float)FS);
    psk_bqc_init_lpf(&lpf_Q, 1, baud * 4.0f, (float)FS);
    psk_timing_init(&timing, SPS);

    float nco_phase = 0.0f;
    float nco_inc   = 2.0f * (float)M_PI * FREQ / (float)FS;

    printf("Symbol# | I_filt   | Q_filt   | phase    | dp       | bit\n");
    printf("--------|----------|----------|----------|----------|----\n");

    float prev_phase = 0.0f;
    int   got_first  = 0;
    int   sym_count  = 0;

    /* Manual varicode accumulation — pending_zero approach */
    uint16_t vacc      = 0;
    int      vacc_len  = 0;
    int      vacc_act  = 0;   /* 1 once first 1-bit seen */
    int      pend_zero = 0;   /* 1 if previous bit was 0 (not yet committed) */
    char     decoded[256];
    int      dec_len = 0;

    for (unsigned n = 0; n < NSAMP; n++) {
        float x   = pcm[n];
        float bp  = psk_bq_step(&bpf, x);

        float phi   = nco_phase;
        float I_raw =  bp * cosf(phi);
        float Q_raw = -bp * sinf(phi);

        float I_filt = psk_bqc_step(&lpf_I, I_raw);
        float Q_filt = psk_bqc_step(&lpf_Q, Q_raw);

        nco_phase += nco_inc;
        while (nco_phase >  (float)M_PI) nco_phase -= 2.0f * (float)M_PI;
        while (nco_phase < -(float)M_PI) nco_phase += 2.0f * (float)M_PI;

        float I_sym, Q_sym;
        if (!psk_timing_step(&timing, I_filt, Q_filt, &I_sym, &Q_sym))
            continue;

        float phase = atan2f(Q_sym, I_sym);

        if (!got_first) {
            prev_phase = phase;
            got_first  = 1;
            if (sym_count <= 60)
                printf("  %4d  | %8.4f | %8.4f | %8.4f | (first)  | -\n",
                       sym_count, I_sym, Q_sym, phase);
        } else {
            float dp = phase - prev_phase;
            while (dp >  (float)M_PI) dp -= 2.0f * (float)M_PI;
            while (dp < -(float)M_PI) dp += 2.0f * (float)M_PI;
            int bit = (fabsf(dp) < (float)M_PI / 2.0f) ? 1 : 0;

            if (sym_count <= 60)
                printf("  %4d  | %8.4f | %8.4f | %8.4f | %8.4f | %d\n",
                       sym_count, I_sym, Q_sym, phase, dp, bit);

            /* Varicode accumulation — pending_zero lookahead */
            if (bit) {
                if (pend_zero) {
                    vacc = (uint16_t)(vacc << 1);  /* commit pending 0 */
                    vacc_len++;
                    pend_zero = 0;
                }
                vacc_act = 1;
                vacc     = (uint16_t)((vacc << 1) | 1);
                vacc_len++;
                if (vacc_len > 14) { vacc = 0; vacc_len = 0; vacc_act = 0; }
            } else {
                if (!vacc_act) {
                    /* ignore leading zeros (preamble) */
                } else if (pend_zero) {
                    /* two consecutive zeros = separator */
                    if (vacc_len > 0 && vacc_len <= 14) {
                        uint8_t bits[16];
                        int i;
                        for (i = vacc_len - 1; i >= 0; i--)
                            bits[vacc_len - 1 - i] = (vacc >> i) & 1;
                        char c = psk_varicode_decode(bits, (size_t)vacc_len);
                        printf("  [decoded char: '%c' (0x%02X), len=%d, val=0x%X]\n",
                               c ? c : '?', (unsigned char)c, vacc_len, vacc);
                        if (c && dec_len < 255) decoded[dec_len++] = c;
                    }
                    vacc = 0; vacc_len = 0; vacc_act = 0; pend_zero = 0;
                } else {
                    pend_zero = 1;  /* hold — might be codeword zero or separator start */
                }
            }

            prev_phase = phase;
        }
        sym_count++;
    }

    decoded[dec_len] = '\0';
    printf("\nDirect-chain demodulation result: '%s'\n", decoded);

    free(pcm);
    return 0;
}
