/*
 * test_modes.c — Quick loopback compile/run test for all 6 modes.
 *
 * Only tests that TX+RX create/destroy work for each mode and that
 * psk_tx_write/read/idle operate without crashing.
 * Full decode accuracy is tested in test_loopback.c.
 */

#include "psk.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static const char *mode_names[] = {
    "BPSK31", "QPSK31", "BPSK63", "BPSK125", "QPSK63", "QPSK125"
};

int main(void)
{
    const unsigned FS   = 8000;
    const float    FREQ = 1500.0f;
    const unsigned NSAMP = FS;   /* 1 second */
    int failures = 0;

    float *pcm = (float *)malloc(NSAMP * sizeof(float));
    assert(pcm);

    for (int m = 0; m < 6; m++) {
        psk_mode_t mode = (psk_mode_t)m;

        /* TX */
        psk_tx_t *tx = psk_tx_create(FS, FREQ, mode);
        if (!tx) {
            printf("FAIL: mode %s: psk_tx_create returned NULL\n", mode_names[m]);
            failures++;
            continue;
        }
        psk_tx_write(tx, "HI", 2);
        size_t n = psk_tx_read(tx, pcm, NSAMP);
        assert(n == NSAMP);

        /* RX */
        psk_rx_t *rx = psk_rx_create(FS, FREQ, mode);
        if (!rx) {
            printf("FAIL: mode %s: psk_rx_create returned NULL\n", mode_names[m]);
            psk_tx_destroy(tx);
            failures++;
            continue;
        }
        psk_rx_feed(rx, pcm, NSAMP);
        psk_rx_reset(rx);
        psk_rx_destroy(rx);
        psk_tx_destroy(tx);

        printf("Mode %-8s  OK\n", mode_names[m]);
    }

    free(pcm);

    /* Test invalid params */
    assert(psk_tx_create(0, 1000.0f, PSK_MODE_BPSK31) == NULL);
    assert(psk_tx_create(8000, 0.0f,  PSK_MODE_BPSK31) == NULL);
    assert(psk_rx_create(0, 1000.0f, PSK_MODE_BPSK31) == NULL);

    if (failures == 0)
        puts("All mode tests PASSED.");
    else
        printf("%d mode test(s) FAILED.\n", failures);

    return failures ? 1 : 0;
}
