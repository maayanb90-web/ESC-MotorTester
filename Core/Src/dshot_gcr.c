#include "dshot_gcr.h"

#include <string.h>

/*
 * Pure bidirectional-DShot RX decoder. No STM32 includes here; this file
 * is compiled both into the firmware and into the host test harness
 * (tools/decode_test.c) so we can verify against captured fixtures
 * before going to the bench.
 *
 * See dshot_gcr.h for the format reference.
 */

#define DSHOT_RX_BITS              21U
#define DSHOT_RX_GCR_BITS          20U
#define DSHOT_RX_INVALID_NIBBLE    0xFFU

/* 5b -> 4b GCR lookup (Bluejay/BLHeli_S table). Codes not listed are
 * invalid and translate to 0xFF. */
static const uint8_t s_gcr5_to_4[32] = {
    [0x19] = 0x0, [0x1B] = 0x1, [0x12] = 0x2, [0x13] = 0x3,
    [0x1D] = 0x4, [0x15] = 0x5, [0x16] = 0x6, [0x17] = 0x7,
    [0x1A] = 0x8, [0x09] = 0x9, [0x0A] = 0xA, [0x0B] = 0xB,
    [0x1E] = 0xC, [0x0D] = 0xD, [0x0E] = 0xE, [0x0F] = 0xF,
    /* Any 5-bit code not listed above defaults to 0x00, which is *not*
     * what we want — patch in the invalid marker at init time. We can't
     * use a designated-init "default" in plain C89/C99, so we provide
     * a small init helper called once at decode time below. */
};

static bool s_gcr_table_ready = false;
static uint8_t s_gcr_table[32];

static void gcr_table_init_once(void)
{
    if (s_gcr_table_ready) return;
    for (int i = 0; i < 32; ++i) {
        s_gcr_table[i] = DSHOT_RX_INVALID_NIBBLE;
    }
    s_gcr_table[0x19] = 0x0; s_gcr_table[0x1B] = 0x1;
    s_gcr_table[0x12] = 0x2; s_gcr_table[0x13] = 0x3;
    s_gcr_table[0x1D] = 0x4; s_gcr_table[0x15] = 0x5;
    s_gcr_table[0x16] = 0x6; s_gcr_table[0x17] = 0x7;
    s_gcr_table[0x1A] = 0x8; s_gcr_table[0x09] = 0x9;
    s_gcr_table[0x0A] = 0xA; s_gcr_table[0x0B] = 0xB;
    s_gcr_table[0x1E] = 0xC; s_gcr_table[0x0D] = 0xD;
    s_gcr_table[0x0E] = 0xE; s_gcr_table[0x0F] = 0xF;
    (void)s_gcr5_to_4; /* keep the literal in the binary for diff/audit */
    s_gcr_table_ready = true;
}

/* --------------------------------------------------------------------------
 * Stage 1: edge timestamps -> per-bit line state.
 *
 * Sample the line at the centre of each bit cell, using the first edge
 * as the t0 reference (line went HIGH->LOW there, so the cell starting
 * at t0 is LOW). Sampling mid-cell rather than at the boundary gives us
 * up to ±0.5 bit-cell of timing slack against ESC clock drift.
 * -------------------------------------------------------------------------- */
uint8_t DShotGcr_EdgesToBits(const uint16_t *edges, uint16_t n_edges,
                             uint16_t bit_ticks, uint8_t *bits_out)
{
    if (n_edges == 0 || bit_ticks == 0 || edges == NULL || bits_out == NULL) {
        return 0;
    }

    uint16_t sample = (uint16_t)(edges[0] + (bit_ticks / 2U));
    uint16_t edge_idx = 1U;
    uint8_t  state    = 0U;    /* line is low for the first cell starting at edges[0] */

    for (uint8_t i = 0; i < DSHOT_RX_BITS; ++i) {
        /* Advance through any edges <= the mid-cell sample point. Each
         * edge flips the recorded line state. The capture timer is
         * 16-bit; we treat (a - b) as a signed delta to handle wrap. */
        while (edge_idx < n_edges) {
            int16_t delta = (int16_t)(edges[edge_idx] - sample);
            if (delta > 0) break;
            state    ^= 1U;
            edge_idx += 1U;
        }
        bits_out[i] = state;
        sample     += bit_ticks;
    }
    return DSHOT_RX_BITS;
}

/* --------------------------------------------------------------------------
 * Stage 2+3: bits -> XOR-decode -> 5b/4b lookup -> 16-bit packet -> CRC.
 * -------------------------------------------------------------------------- */
bool DShotGcr_BitsToFrame(const uint8_t *bits, DShotGcrFrame *out)
{
    if (bits == NULL || out == NULL) return false;
    gcr_table_init_once();
    memset(out, 0, sizeof(*out));

    /* XOR-decode. The line was HIGH (= 1) before the first cell; each
     * subsequent decoded bit is the XOR of two consecutive line states.
     * The leading decoded bit is always 1 (the start-of-frame edge);
     * we drop it. */
    uint8_t decoded[DSHOT_RX_BITS];
    uint8_t prev = 1U;
    for (uint8_t i = 0; i < DSHOT_RX_BITS; ++i) {
        decoded[i] = bits[i] ^ prev;
        prev       = bits[i];
    }

    /* Sanity: the leading decoded bit must be 1 (start transition). If
     * it isn't, the timing recovery in Stage 1 was misaligned. */
    if (decoded[0] != 1U) {
        return false;
    }

    /* Pack the remaining 20 bits into 4 GCR symbols, MSB first. */
    uint8_t gcr[4] = {0};
    for (uint8_t n = 0; n < 4; ++n) {
        uint8_t sym = 0;
        for (uint8_t b = 0; b < 5; ++b) {
            sym = (uint8_t)((sym << 1) | decoded[1 + n * 5 + b]);
        }
        gcr[n] = sym;
    }

    /* 5b -> 4b lookup. */
    uint8_t nibbles[4];
    for (uint8_t n = 0; n < 4; ++n) {
        uint8_t v = s_gcr_table[gcr[n] & 0x1F];
        if (v == DSHOT_RX_INVALID_NIBBLE) {
            return false;
        }
        nibbles[n] = v;
    }

    uint16_t pkt = (uint16_t)((nibbles[0] << 12) | (nibbles[1] << 8) |
                              (nibbles[2] << 4)  |  nibbles[3]);
    out->raw_packet = pkt;

    /* CRC: bottom nibble = (~XOR of upper 3 nibbles) & 0x0F. */
    uint16_t v12 = (uint16_t)(pkt >> 4);
    uint8_t  crc_recv = (uint8_t)(pkt & 0x0F);
    uint8_t  crc_calc = (uint8_t)((~(v12 ^ (v12 >> 4) ^ (v12 >> 8))) & 0x0F);
    if (crc_recv != crc_calc) {
        return false;
    }

    /* Extract mantissa/exponent. v12 == 0x0FFF is the "motor stopped"
     * sentinel (no spin -> reported eRPM is zero). */
    if (v12 == 0x0FFF) {
        out->valid         = true;
        out->motor_stopped = true;
        return true;
    }

    /* Bluejay / Betaflight RPM-Filter packing: top 3 bits = exponent,
     * low 9 bits = mantissa. period_us = mantissa << exponent. (Some
     * older 4+8 BLHeli variants exist; this codepath assumes the modern
     * 3+9 scheme used by Bluejay v0.20+, which the production ESC runs.) */
    out->exponent  = (uint8_t)((v12 >> 9) & 0x07);
    out->mantissa  = (uint16_t)(v12 & 0x01FF);
    out->period_us = (uint32_t)out->mantissa << out->exponent;
    out->valid     = (out->period_us != 0U);
    return out->valid;
}

bool DShotGcr_Decode(const uint16_t *edges, uint16_t n_edges,
                     uint16_t bit_ticks, DShotGcrFrame *out)
{
    uint8_t bits[DSHOT_RX_BITS];
    if (DShotGcr_EdgesToBits(edges, n_edges, bit_ticks, bits) != DSHOT_RX_BITS) {
        return false;
    }
    return DShotGcr_BitsToFrame(bits, out);
}

uint32_t DShotGcr_PeriodToRpm(uint32_t period_us, uint8_t pole_count)
{
    if (period_us == 0U || pole_count == 0U) return 0U;
    /* eRPM = 60_000_000 / period_us;  mechanical RPM = eRPM * 2 / poles. */
    uint32_t erpm = 60000000U / period_us;
    return (erpm * 2U) / (uint32_t)pole_count;
}
