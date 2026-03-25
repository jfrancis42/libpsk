/*
 * test_loopback.c — TX→RX loopback test for BPSK31.
 *
 * Generates PSK31 audio, feeds it directly to the decoder, and verifies
 * the decoded text contains the transmitted message.
 *
 * This is a loopback with no channel distortion — ideal conditions.
 * The decoder gets perfectly modulated audio at 0 dB SNR (noise-free).
 */

#include "psk.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char decoded_buf[1024];
static int  decoded_len = 0;

static void on_char(char c, void *ud)
{
    (void)ud;
    if (decoded_len < (int)sizeof(decoded_buf) - 1)
        decoded_buf[decoded_len++] = c;
}

static int run_loopback(psk_mode_t mode, const char *modename)
{
    const unsigned FS   = 8000;
    const float    FREQ = 1000.0f;
    /* 5 seconds: enough for preamble + message + postamble + decoder latency */
    const unsigned NSAMP = FS * 5;
    const char *msg = "HELLO PSK";

    float *pcm = (float *)malloc(NSAMP * sizeof(float));
    if (!pcm) return 1;

    decoded_len = 0;
    memset(decoded_buf, 0, sizeof(decoded_buf));

    /* TX */
    psk_tx_t *tx = psk_tx_create(FS, FREQ, mode);
    assert(tx);
    psk_tx_write(tx, msg, strlen(msg));
    size_t n = psk_tx_read(tx, pcm, NSAMP);
    assert(n == NSAMP);
    psk_tx_destroy(tx);

    /* RX */
    psk_rx_t *rx = psk_rx_create(FS, FREQ, mode);
    assert(rx);
    psk_rx_set_char_callback(rx, on_char, NULL);

    /* Feed in blocks of 256 samples */
    for (unsigned pos = 0; pos < NSAMP; pos += 256) {
        unsigned blk = ((pos + 256 <= NSAMP) ? 256 : (NSAMP - pos));
        psk_rx_feed(rx, pcm + pos, blk);
    }

    psk_rx_destroy(rx);
    free(pcm);

    decoded_buf[decoded_len] = '\0';
    printf("Mode %-8s  TX:'%s'  RX:'%s'", modename, msg, decoded_buf);

    /* Check that the target message appears somewhere in decoded output.
     * We allow leading/trailing garbage from preamble/decoder startup. */
    if (strstr(decoded_buf, msg) != NULL) {
        printf("  PASS\n");
        return 0;
    } else {
        printf("  FAIL (message not found)\n");
        return 1;
    }
}

int main(void)
{
    int failures = 0;

    failures += run_loopback(PSK_MODE_BPSK31,  "BPSK31");
    failures += run_loopback(PSK_MODE_BPSK63,  "BPSK63");
    failures += run_loopback(PSK_MODE_BPSK125, "BPSK125");

    if (failures == 0)
        puts("All loopback tests PASSED.");
    else
        printf("%d loopback test(s) FAILED.\n", failures);

    return failures ? 1 : 0;
}
