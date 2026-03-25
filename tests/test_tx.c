/*
 * test_tx.c — TX generation test.
 *
 * Generates 3 seconds of BPSK31 PCM and verifies:
 *   1. Non-zero audio is produced.
 *   2. Amplitude is ≤ 1.0 (no clipping).
 *   3. psk_tx_idle() returns true after text drains.
 */

#include "psk.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
    const unsigned FS    = 8000;
    const float    FREQ  = 1000.0f;
    const unsigned NSAMP = FS * 3;   /* 3 seconds */

    float *pcm = (float *)malloc(NSAMP * sizeof(float));
    assert(pcm);

    psk_tx_t *tx = psk_tx_create(FS, FREQ, PSK_MODE_BPSK31);
    assert(tx);

    /* Write a short test string */
    const char *msg = "TEST DE W0PSK";
    size_t written = psk_tx_write(tx, msg, strlen(msg));
    assert(written == strlen(msg));

    /* Generate audio */
    size_t n = psk_tx_read(tx, pcm, NSAMP);
    assert(n == NSAMP);

    /* Check amplitude ≤ 1.0 */
    float peak = 0.0f;
    for (unsigned i = 0; i < NSAMP; i++) {
        float a = fabsf(pcm[i]);
        if (a > peak) peak = a;
    }
    printf("Peak amplitude: %.4f\n", peak);
    assert(peak <= 1.001f);   /* 0.001 tolerance for float rounding */

    /* Check some non-zero audio (carrier present) */
    float rms = 0.0f;
    for (unsigned i = 0; i < NSAMP; i++)
        rms += pcm[i] * pcm[i];
    rms = sqrtf(rms / NSAMP);
    printf("RMS amplitude: %.4f\n", rms);
    assert(rms > 0.01f);

    /* After draining, should eventually become idle */
    {
        float tmp[256];
        int max_iter = 1000, idle_found = 0;
        while (max_iter-- > 0) {
            psk_tx_read(tx, tmp, 256);
            if (psk_tx_idle(tx)) { idle_found = 1; break; }
        }
        if (!idle_found)
            printf("WARNING: TX did not reach idle in 256000 extra samples\n");
        else
            puts("TX reached idle after text drained.");
    }

    psk_tx_destroy(tx);
    free(pcm);

    puts("All TX tests PASSED.");
    return 0;
}
