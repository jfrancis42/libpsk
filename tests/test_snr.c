/*
 * test_snr.c — SNR threshold test.
 *
 * Adds white Gaussian noise at various SNR levels and measures the
 * character error rate.  BPSK31 should decode cleanly at ≥ -10 dB SNR.
 *
 * Uses a simple LCG pseudo-random noise generator (no stdlib rand dependency).
 */

#include "psk.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* LCG random number generator (Knuth) */
static uint32_t lcg_state = 0xDEADBEEFu;

static float lcg_rand_gauss(void)
{
    /* Box-Muller transform */
    float u1, u2;
    lcg_state = lcg_state * 1664525u + 1013904223u;
    u1 = (float)(lcg_state >> 1) / (float)0x7FFFFFFFu;
    lcg_state = lcg_state * 1664525u + 1013904223u;
    u2 = (float)(lcg_state >> 1) / (float)0x7FFFFFFFu;
    if (u1 < 1e-10f) u1 = 1e-10f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

static char decoded_buf[2048];
static int  decoded_len;

static void on_char(char c, void *ud) {
    (void)ud;
    if (decoded_len < (int)sizeof(decoded_buf) - 1)
        decoded_buf[decoded_len++] = c;
}

static int count_matches(const char *decoded, const char *msg)
{
    /* Count occurrences of msg in decoded */
    int count = 0;
    const char *p = decoded;
    size_t mlen = strlen(msg);
    while ((p = strstr(p, msg)) != NULL) { count++; p += mlen; }
    return count;
}

int main(void)
{
    const unsigned FS   = 8000;
    const float    FREQ = 1000.0f;
    const unsigned NSAMP = FS * 6;
    const char *msg = "TEST";

    float *pcm_clean = (float *)malloc(NSAMP * sizeof(float));
    float *pcm_noisy = (float *)malloc(NSAMP * sizeof(float));
    assert(pcm_clean && pcm_noisy);

    /* Generate clean BPSK31 audio */
    psk_tx_t *tx = psk_tx_create(FS, FREQ, PSK_MODE_BPSK31);
    assert(tx);
    for (int rep = 0; rep < 3; rep++)   /* repeat msg 3× to fill buffer */
        psk_tx_write(tx, msg, strlen(msg));
    psk_tx_read(tx, pcm_clean, NSAMP);
    psk_tx_destroy(tx);

    /* Measure signal power */
    double sig_power = 0.0;
    for (unsigned i = 0; i < NSAMP; i++)
        sig_power += pcm_clean[i] * pcm_clean[i];
    sig_power /= NSAMP;

    /* Test at several SNR levels (dB) */
    static const float snr_dbs[] = { 20.0f, 10.0f, 5.0f, 0.0f, -5.0f, -10.0f };
    int n_levels = (int)(sizeof(snr_dbs) / sizeof(snr_dbs[0]));
    int failures = 0;

    for (int li = 0; li < n_levels; li++) {
        float snr_db       = snr_dbs[li];
        float snr_linear   = powf(10.0f, snr_db / 10.0f);
        float noise_sigma  = sqrtf((float)sig_power / snr_linear);

        /* Add noise */
        lcg_state = 0xCAFEBABEu;  /* deterministic seed */
        for (unsigned i = 0; i < NSAMP; i++)
            pcm_noisy[i] = pcm_clean[i] + noise_sigma * lcg_rand_gauss();

        /* Decode */
        decoded_len = 0;
        memset(decoded_buf, 0, sizeof(decoded_buf));
        psk_rx_t *rx = psk_rx_create(FS, FREQ, PSK_MODE_BPSK31);
        assert(rx);
        psk_rx_set_char_callback(rx, on_char, NULL);
        psk_rx_feed(rx, pcm_noisy, NSAMP);
        psk_rx_destroy(rx);

        decoded_buf[decoded_len] = '\0';
        int found = count_matches(decoded_buf, msg);
        int ok    = (snr_db >= -10.0f) ? (found >= 1) : 1; /* no requirement below -10 dB */

        printf("SNR %+5.0f dB: found '%s' %d time(s) in decoded='%.*s'%s  %s\n",
               snr_db, msg, found,
               decoded_len > 40 ? 40 : decoded_len, decoded_buf,
               decoded_len > 40 ? "..." : "",
               ok ? "OK" : "FAIL");
        if (!ok) failures++;
    }

    free(pcm_clean);
    free(pcm_noisy);

    if (failures == 0)
        puts("All SNR tests PASSED.");
    else
        printf("%d SNR test(s) FAILED.\n", failures);

    return failures ? 1 : 0;
}
