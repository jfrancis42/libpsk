/*
 * test_debug2.c — Debug BPSK63/125 demodulation.
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

static void run_debug(psk_mode_t mode, const char *name, float baud, int sps)
{
    const unsigned FS   = 8000;
    const float    FREQ = 1000.0f;
    const unsigned NSAMP = FS * 3;

    float *pcm = (float *)malloc(NSAMP * sizeof(float));
    if (!pcm) return;

    psk_tx_t *tx = psk_tx_create(FS, FREQ, mode);
    psk_tx_write(tx, "HELLO PSK", 9);
    psk_tx_read(tx, pcm, NSAMP);
    psk_tx_destroy(tx);

    psk_bq_t  bpf;
    psk_bqc_t lpf_I, lpf_Q;
    psk_timing_t timing;

    psk_bq_init_bpf(&bpf, FREQ, baud * 8.0f, (float)FS);
    psk_bqc_init_lpf(&lpf_I, 1, baud * 4.0f, (float)FS);
    psk_bqc_init_lpf(&lpf_Q, 1, baud * 4.0f, (float)FS);
    psk_timing_init(&timing, sps);

    float nco_phase = 0.0f;
    float nco_inc   = 2.0f * (float)M_PI * FREQ / (float)FS;

    printf("\n=== %s (baud=%.1f, sps=%d) ===\n", name, baud, sps);
    printf("Symbol# | I_filt   | phase    | dp       | bit\n");
    printf("--------|----------|----------|----------|----\n");

    float prev_phase = 0.0f;
    int   got_first  = 0;
    int   sym_count  = 0;

    uint16_t vacc     = 0;
    int      vacc_len = 0;
    int      vacc_act = 0;
    int      pend_z   = 0;
    char     decoded[256];
    int      dec_len  = 0;

    for (unsigned n = 0; n < NSAMP; n++) {
        float x   = pcm[n];
        float bp  = psk_bq_step(&bpf, x);
        float I_raw =  bp * cosf(nco_phase);
        float Q_raw = -bp * sinf(nco_phase);
        float I_filt = psk_bqc_step(&lpf_I, I_raw);
        float Q_filt = psk_bqc_step(&lpf_Q, Q_raw);
        nco_phase += nco_inc;
        while (nco_phase > (float)M_PI)  nco_phase -= 2.0f * (float)M_PI;
        while (nco_phase < -(float)M_PI) nco_phase += 2.0f * (float)M_PI;

        float I_sym, Q_sym;
        if (!psk_timing_step(&timing, I_filt, Q_filt, &I_sym, &Q_sym))
            continue;

        float phase = atan2f(Q_sym, I_sym);

        if (!got_first) {
            prev_phase = phase;
            got_first  = 1;
            if (sym_count <= 50)
                printf("  %4d  | %8.4f | %8.4f | (first)  | -\n",
                       sym_count, I_sym, phase);
        } else {
            float dp = phase - prev_phase;
            while (dp >  (float)M_PI) dp -= 2.0f * (float)M_PI;
            while (dp < -(float)M_PI) dp += 2.0f * (float)M_PI;
            int bit = (fabsf(dp) < (float)M_PI / 2.0f) ? 1 : 0;

            if (sym_count <= 50)
                printf("  %4d  | %8.4f | %8.4f | %8.4f | %d\n",
                       sym_count, I_sym, phase, dp, bit);

            if (bit) {
                if (pend_z) {
                    vacc = (uint16_t)(vacc << 1); vacc_len++; pend_z = 0;
                }
                vacc_act = 1;
                vacc = (uint16_t)((vacc << 1) | 1);
                vacc_len++;
                if (vacc_len > 14) { vacc = 0; vacc_len = 0; vacc_act = 0; }
            } else {
                if (!vacc_act) {
                } else if (pend_z) {
                    if (vacc_len > 0 && vacc_len <= 14) {
                        uint8_t bits[16]; int i;
                        for (i = vacc_len - 1; i >= 0; i--)
                            bits[vacc_len-1-i] = (vacc >> i) & 1;
                        char c = psk_varicode_decode(bits, (size_t)vacc_len);
                        printf("    CHAR '%c' (len=%d val=0x%X)\n", c?c:'?', vacc_len, vacc);
                        if (c && dec_len < 255) decoded[dec_len++] = c;
                    }
                    vacc = 0; vacc_len = 0; vacc_act = 0; pend_z = 0;
                } else {
                    pend_z = 1;
                }
            }
            prev_phase = phase;
        }
        sym_count++;
    }

    decoded[dec_len] = '\0';
    printf("Result: '%s'\n", decoded);
    free(pcm);
}

int main(void)
{
    run_debug(PSK_MODE_BPSK63,  "BPSK63",  62.5f,  128);
    run_debug(PSK_MODE_BPSK125, "BPSK125", 125.0f, 64);
    return 0;
}
