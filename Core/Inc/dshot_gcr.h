#ifndef DSHOT_GCR_H
#define DSHOT_GCR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Pure bidirectional-DShot RX decoder.
 *
 * No STM32 includes — this translation unit compiles on the host too,
 * which is the point: the bit-recovery and 5b/4b decode logic is
 * exercised by tools/decode_test.c against captured fixtures before any
 * firmware reaches the bench.
 *
 * The on-device side (Core/Src/dshot.c) is responsible for capturing
 * edge timestamps via TIM1 input capture + DMA, and calling
 * DShotGcr_Decode() per channel.
 *
 * --------------------------------------------------------------------------
 * Format reference (Bluejay v0.20 / BLHeli_S bidirectional DShot)
 * --------------------------------------------------------------------------
 *
 * On the wire the ESC sends 21 bit periods, MSB first, at the same nominal
 * rate as the TX bit clock (DShot300 -> 3.33 us / bit). The line idles HIGH
 * and the ESC opens transmission by pulling LOW for the first bit cell.
 *
 * Encoding pipeline (ESC side, for reference; we invert it on RX):
 *
 *   1. 12-bit eRPM-period value v12: top 4 bits = exponent, low 8 bits
 *      = mantissa. period_us = mantissa << exponent. Special value
 *      v12 == 0x0FFF means "motor stopped / eRPM = 0".
 *   2. 4-bit CRC: crc = (~(v12 ^ (v12 >> 4) ^ (v12 >> 8))) & 0x0F.
 *   3. 16-bit packet: pkt = (v12 << 4) | crc.
 *   4. Split into 4 nibbles, MSB first; encode each with a 4b/5b GCR
 *      table -> 20 raw bits.
 *   5. XOR-encode: out[i] = raw[i] XOR out[i-1] (with the initial line
 *      state HIGH). This makes each "1" in the raw stream a line
 *      transition. A leading edge is implicit (line goes high->low at
 *      the start).
 *
 * Decoding inverts these steps; see DShotGcr_Decode() below.
 * --------------------------------------------------------------------------
 */

typedef struct {
    bool     valid;         /* CRC OK and value within plausible range */
    bool     motor_stopped; /* v12 == 0x0FFF reported by ESC           */
    uint8_t  exponent;      /* 3 bits (Bluejay / Betaflight RPM-Filter) */
    uint16_t mantissa;      /* 9 bits                                   */
    uint32_t period_us;     /* mantissa << exponent (0 if motor_stopped)*/
    uint16_t raw_packet;    /* the 16-bit packet, for diagnostics       */
} DShotGcrFrame;

/*
 * Stage 1: convert a sequence of edge timestamps into per-bit line states.
 *
 *   edges            : monotonically increasing tick counts of transitions.
 *                      edges[0] is the first HIGH->LOW transition that
 *                      starts the frame.
 *   n_edges          : number of edges captured (<= some upper bound the
 *                      caller knows; we only need the first ~21).
 *   bit_ticks        : nominal ticks per bit cell, in the same unit as edges.
 *   bits_out         : 21-entry buffer for the recovered line state per
 *                      bit cell (0 = low, 1 = high).
 *
 * Returns the number of bit cells decoded (21 on a clean frame). Returns
 * 0 if the input is unusable (no edges, or bit_ticks == 0).
 */
uint8_t DShotGcr_EdgesToBits(const uint16_t *edges, uint16_t n_edges,
                             uint16_t bit_ticks, uint8_t *bits_out);

/*
 * Stage 2+3: from 21 bit states to a validated DShotGcrFrame.
 *
 * Performs the XOR-decode, 5b/4b lookup, CRC check, and the
 * mantissa/exponent split.
 */
bool DShotGcr_BitsToFrame(const uint8_t *bits, DShotGcrFrame *out);

/*
 * One-shot convenience: edges -> validated frame in one call.
 */
bool DShotGcr_Decode(const uint16_t *edges, uint16_t n_edges,
                     uint16_t bit_ticks, DShotGcrFrame *out);

/*
 * eRPM-period -> mechanical RPM, for the configured motor pole count.
 *
 *   period_us  : as returned by DShotGcr_Decode().
 *   pole_count : number of magnetic poles (PRD: 14).
 *
 * Returns 0 for period_us == 0 (motor stopped) or any pathological input.
 */
uint32_t DShotGcr_PeriodToRpm(uint32_t period_us, uint8_t pole_count);

#endif /* DSHOT_GCR_H */
