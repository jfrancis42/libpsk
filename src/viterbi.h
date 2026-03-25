/*
 * viterbi.h — Rate-1/2, K=5 convolutional codec for QPSK31.
 *
 * Generator polynomials (G3PLX QPSK31 spec):
 *   G1 = 0x17 = 10111 binary   (industry standard K=5, rate 1/2)
 *   G2 = 0x19 = 11001 binary
 *
 * The encoder processes 1 input bit → 2 output bits.
 * The decoder is a 16-state soft-decision Viterbi decoder.
 * Traceback depth: 32 bits (6.4 × constraint length).
 *
 * MIT License. Copyright (c) 2025.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Encoder ────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t sr;   /* shift register, 4 bits (K-1 = 4 memory stages) */
} psk_conv_t;

void psk_conv_reset(psk_conv_t *c);

/* Encode one bit: bit → (out0, out1).  Both out0/out1 are 0 or 1. */
void psk_conv_encode(psk_conv_t *c, int bit, int *out0, int *out1);

/* Encode and flush K-1 tail bits (brings encoder back to state 0).
 * Appends 4 extra pairs (8 bits) to out[].
 * Returns 8 (the number of bits written). */
int psk_conv_flush(psk_conv_t *c, int *out0, int *out1, int n_flush);

/* ── Decoder ────────────────────────────────────────────────────────────── */

#define PSK_VITERBI_STATES   16   /* 2^(K-1) = 2^4 = 16 */
#define PSK_VITERBI_DEPTH    32   /* traceback depth in symbols */
#define PSK_VITERBI_LOOKAHEAD (PSK_VITERBI_STATES * PSK_VITERBI_DEPTH)

typedef struct {
    float  path_metric[PSK_VITERBI_STATES];
    /* Traceback: tb[depth][state] = predecessor state */
    uint8_t tb_state[PSK_VITERBI_DEPTH][PSK_VITERBI_STATES];
    uint8_t tb_bit  [PSK_VITERBI_DEPTH][PSK_VITERBI_STATES];
    int    tb_idx;        /* circular traceback index */
    int    count;         /* symbols fed since reset */

    /* Output FIFO (decoded bits, after traceback delay) */
    uint8_t out_buf[PSK_VITERBI_DEPTH];
    int     out_r, out_w;
} psk_viterbi_t;

void psk_viterbi_reset(psk_viterbi_t *v);

/*
 * Feed one soft symbol pair (s0, s1 are soft-decision LLR values).
 * Positive = more likely "1", negative = more likely "0".
 * Returns decoded bit (0 or 1) if available after traceback, else -1.
 */
int psk_viterbi_decode(psk_viterbi_t *v, float s0, float s1);

#ifdef __cplusplus
}
#endif
