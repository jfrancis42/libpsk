/*
 * test_varicode.c — Varicode encode/decode round-trip test.
 *
 * Encodes all 128 ASCII characters, verifies the 00 separator is appended,
 * verifies the decoded character matches, and checks that no codeword
 * contains an internal "00" bit pair.
 */

#include "psk.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    int c, i, failures = 0;

    for (c = 0; c < 128; c++) {
        uint8_t bits[16];
        int     n = psk_varicode_encode((char)c, bits);
        char    decoded;
        int     j;

        /* Must have at least 3 bits (1 code bit + 2 stop bits) */
        if (n < 3) {
            printf("FAIL: char %d: encode returned %d bits\n", c, n);
            failures++;
            continue;
        }

        /* Last two bits must be 00 (inter-character separator) */
        if (bits[n - 2] != 0 || bits[n - 1] != 0) {
            printf("FAIL: char %d: missing 00 separator (bits[%d]=%d bits[%d]=%d)\n",
                   c, n-2, bits[n-2], n-1, bits[n-1]);
            failures++;
            continue;
        }

        /* No internal "00" in the code portion (bits[0..n-3]) */
        for (j = 0; j < n - 3; j++) {
            if (bits[j] == 0 && bits[j+1] == 0) {
                printf("FAIL: char %d: internal 00 at positions %d,%d\n", c, j, j+1);
                failures++;
                break;
            }
        }

        /* First bit must be 1 */
        if (bits[0] != 1) {
            printf("FAIL: char %d: first bit is 0\n", c);
            failures++;
            continue;
        }

        /* Decode (pass only the code bits, not the 00 separator) */
        decoded = psk_varicode_decode(bits, (size_t)(n - 2));
        if ((unsigned char)decoded != (unsigned char)c) {
            printf("FAIL: char %d: encode→decode round-trip: got %d\n",
                   c, (unsigned char)decoded);
            failures++;
        }
    }

    /* Test a few known values */
    {
        uint8_t bits[16];
        int n;

        /* Space (0x20) should encode to 1 code bit + 2 separator bits */
        n = psk_varicode_encode(' ', bits);
        assert(n == 3);
        assert(bits[0] == 1);
        assert(bits[1] == 0);
        assert(bits[2] == 0);

        /* 'e' (0x65) should encode to 2 code bits + 2 separator = 4 total */
        n = psk_varicode_encode('e', bits);
        assert(n == 4);
        assert(bits[0] == 1);
        assert(bits[1] == 1);
        assert(bits[2] == 0);
        assert(bits[3] == 0);
    }

    /* Verify decode of unknown pattern returns 0 */
    {
        uint8_t bad[4] = {0, 0, 1, 0};
        char r = psk_varicode_decode(bad, 4);
        assert(r == 0);
    }

    if (failures == 0)
        puts("All varicode tests PASSED.");
    else
        printf("%d varicode tests FAILED.\n", failures);

    return failures ? 1 : 0;
}
