/*
 * viterbi.c — Rate-1/2, K=5 convolutional codec.
 *
 * MIT License. Copyright (c) 2025.
 */

#include "viterbi.h"
#include <string.h>
#include <float.h>

/* Generator polynomials */
#define G1 0x17   /* 10111 */
#define G2 0x19   /* 11001 */

/* Compute parity (XOR all set bits) of x */
static inline int parity(uint8_t x)
{
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1;
}

/* ── Encoder ────────────────────────────────────────────────────────────── */

void psk_conv_reset(psk_conv_t *c)
{
    c->sr = 0;
}

void psk_conv_encode(psk_conv_t *c, int bit, int *out0, int *out1)
{
    /* Shift new bit into register (K-1 = 4 bits) */
    c->sr = (uint8_t)(((c->sr >> 1) | ((bit & 1) << 3)) & 0x0F);
    /* Compute outputs: the input is the MSB of the 5-bit word [bit, sr[3..0]] */
    uint8_t word = (uint8_t)((bit << 4) | c->sr);
    *out0 = parity(word & G1);
    *out1 = parity(word & G2);
}

int psk_conv_flush(psk_conv_t *c, int *out0, int *out1, int n_flush)
{
    int i;
    for (i = 0; i < n_flush; i++)
        psk_conv_encode(c, 0, &out0[i], &out1[i]);
    return n_flush;
}

/* ── Decoder ────────────────────────────────────────────────────────────── */

/* Pre-compute expected output bits for each (state, input_bit) pair. */
static uint8_t enc_out0[PSK_VITERBI_STATES][2]; /* [state][bit] */
static uint8_t enc_out1[PSK_VITERBI_STATES][2];
static uint8_t enc_next[PSK_VITERBI_STATES][2]; /* next state */
static int     trellis_ready = 0;

static void build_trellis(void)
{
    int s, b;
    if (trellis_ready) return;
    for (s = 0; s < PSK_VITERBI_STATES; s++) {
        for (b = 0; b < 2; b++) {
            /* State = 4-bit shift register (sr[3..0]).  New bit goes in at bit 3. */
            uint8_t word = (uint8_t)((b << 4) | s);
            enc_out0[s][b] = parity(word & G1);
            enc_out1[s][b] = parity(word & G2);
            enc_next[s][b] = (uint8_t)(((s >> 1) | (b << 3)) & 0x0F);
        }
    }
    trellis_ready = 1;
}

void psk_viterbi_reset(psk_viterbi_t *v)
{
    int i;
    build_trellis();
    for (i = 0; i < PSK_VITERBI_STATES; i++)
        v->path_metric[i] = (i == 0) ? 0.0f : 1e30f;
    memset(v->tb_state, 0, sizeof(v->tb_state));
    memset(v->tb_bit,   0, sizeof(v->tb_bit));
    v->tb_idx = 0;
    v->count  = 0;
    v->out_r  = 0;
    v->out_w  = 0;
}

int psk_viterbi_decode(psk_viterbi_t *v, float s0, float s1)
{
    float new_metric[PSK_VITERBI_STATES];
    int   s, b;
    int   best_state;
    float best_m;

    build_trellis();

    for (s = 0; s < PSK_VITERBI_STATES; s++)
        new_metric[s] = 1e30f;

    for (s = 0; s < PSK_VITERBI_STATES; s++) {
        if (v->path_metric[s] >= 1e29f) continue;  /* pruned state */
        for (b = 0; b < 2; b++) {
            int   ns  = enc_next[s][b];
            float e0  = (enc_out0[s][b] ? 1.0f : -1.0f) - s0;
            float e1  = (enc_out1[s][b] ? 1.0f : -1.0f) - s1;
            float m   = v->path_metric[s] + e0 * e0 + e1 * e1;
            if (m < new_metric[ns]) {
                new_metric[ns]              = m;
                v->tb_state[v->tb_idx][ns]  = (uint8_t)s;
                v->tb_bit  [v->tb_idx][ns]  = (uint8_t)b;
            }
        }
    }

    /* Copy new metrics */
    for (s = 0; s < PSK_VITERBI_STATES; s++)
        v->path_metric[s] = new_metric[s];

    /* Normalize to prevent overflow (subtract minimum) */
    {
        float min_m = v->path_metric[0];
        for (s = 1; s < PSK_VITERBI_STATES; s++)
            if (v->path_metric[s] < min_m) min_m = v->path_metric[s];
        for (s = 0; s < PSK_VITERBI_STATES; s++)
            v->path_metric[s] -= min_m;
    }

    v->count++;
    v->tb_idx = (v->tb_idx + 1) % PSK_VITERBI_DEPTH;

    /* Traceback once per symbol after initial fill */
    if (v->count >= PSK_VITERBI_DEPTH) {
        /* Find best ending state */
        best_state = 0;
        best_m     = v->path_metric[0];
        for (s = 1; s < PSK_VITERBI_STATES; s++) {
            if (v->path_metric[s] < best_m) {
                best_m     = v->path_metric[s];
                best_state = s;
            }
        }

        /* Trace back PSK_VITERBI_DEPTH steps */
        {
            int idx   = (v->tb_idx - 1 + PSK_VITERBI_DEPTH) % PSK_VITERBI_DEPTH;
            int state = best_state;
            int d;
            uint8_t bits[PSK_VITERBI_DEPTH];
            for (d = PSK_VITERBI_DEPTH - 1; d >= 0; d--) {
                bits[d] = v->tb_bit[idx][state];
                state   = v->tb_state[idx][state];
                idx     = (idx - 1 + PSK_VITERBI_DEPTH) % PSK_VITERBI_DEPTH;
            }
            /* The oldest bit is bits[0] — put in output FIFO */
            v->out_buf[v->out_w % PSK_VITERBI_DEPTH] = bits[0];
            v->out_w++;
        }
    }

    /* Return next decoded bit if available */
    if (v->out_w != v->out_r) {
        int bit = v->out_buf[v->out_r % PSK_VITERBI_DEPTH];
        v->out_r++;
        return bit;
    }
    return -1;
}
