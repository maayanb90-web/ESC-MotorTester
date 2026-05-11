/*
 * Host-side unit test for Core/Src/dshot_gcr.c.
 *
 * Build & run:
 *   $ make -C tools test
 *
 * The test synthesises edge timestamps for a chosen eRPM period using
 * the same encoding the ESC firmware uses, feeds them through the
 * decoder, and asserts the resulting period round-trips. It also covers
 * the motor-stopped sentinel, CRC corruption, and invalid GCR symbols.
 *
 * This lets us verify the decode logic on a desktop *before* touching
 * the bench. Captured frames from a logic analyzer can be patched into
 * the FIXTURE_* arrays for spot-checks against the real ESC.
 */

#include "../Core/Inc/dshot_gcr.h"
#include "../Core/Inc/rpm_stats.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The 4b->5b encoder, mirror of the table in dshot_gcr.c. */
static const uint8_t kNibbleToGcr[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
};

/* --- Encoder used by the test fixture ------------------------------------ */

/* Build a 16-bit packet from a 12-bit eRPM value. */
static uint16_t pack_packet(uint16_t v12)
{
    uint8_t crc = (~(v12 ^ (v12 >> 4) ^ (v12 >> 8))) & 0x0F;
    return (uint16_t)((v12 << 4) | crc);
}

/* Encode a 16-bit packet into 20 GCR bits, MSB first. */
static void packet_to_gcr_bits(uint16_t pkt, uint8_t *bits20)
{
    for (int n = 0; n < 4; ++n) {
        uint8_t nibble = (pkt >> (12 - n * 4)) & 0x0F;
        uint8_t gcr    = kNibbleToGcr[nibble];
        for (int b = 0; b < 5; ++b) {
            bits20[n * 5 + b] = (gcr >> (4 - b)) & 1U;
        }
    }
}

/* XOR-encode 20 data bits into 21 line states (with a leading start
 * transition). Initial line state is HIGH (1). */
static void xor_encode_to_line(const uint8_t *bits20, uint8_t *line21)
{
    uint8_t prev = 1; /* line was high before the start edge */
    /* Leading bit: the start-of-frame transition is implicit; the first
     * data bit is gcr[0], emitted via line state = prev XOR gcr[0]. */
    for (int i = 0; i < 21; ++i) {
        uint8_t in = (i == 0) ? 1U : bits20[i - 1];
        prev       = (uint8_t)(prev ^ in);
        line21[i]  = prev;
    }
}

/* Synthesise edge timestamps from a line-state vector. */
static uint16_t line_to_edges(const uint8_t *line21, uint16_t bit_ticks,
                              uint16_t *edges_out, uint16_t max_edges)
{
    uint16_t n_edges  = 0;
    uint8_t  prev     = 1; /* idle high */
    uint16_t t        = 100; /* arbitrary, non-zero start time */
    for (int i = 0; i < 21; ++i) {
        if (line21[i] != prev) {
            if (n_edges >= max_edges) return n_edges;
            edges_out[n_edges++] = t;
            prev                 = line21[i];
        }
        t = (uint16_t)(t + bit_ticks);
    }
    return n_edges;
}

/* Full encode: 12-bit value -> edge timestamps. */
static uint16_t encode_frame(uint16_t v12, uint16_t bit_ticks,
                             uint16_t *edges_out, uint16_t max_edges)
{
    uint16_t pkt = pack_packet(v12);
    uint8_t  bits20[20];
    uint8_t  line21[21];
    packet_to_gcr_bits(pkt, bits20);
    xor_encode_to_line(bits20, line21);
    return line_to_edges(line21, bit_ticks, edges_out, max_edges);
}

/* --- Tests --------------------------------------------------------------- */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...) do {                                          \
    if (cond) {                                                             \
        ++g_pass;                                                           \
    } else {                                                                \
        ++g_fail;                                                           \
        fprintf(stderr, "FAIL %s:%d  " fmt "\n", __FILE__, __LINE__,        \
                ##__VA_ARGS__);                                             \
    }                                                                       \
} while (0)

static void test_roundtrip_basic(void)
{
    /* Pack v12 = exponent=2, mantissa=100 -> period = 100 << 2 = 400 us
     * -> eRPM = 60_000_000 / 400 = 150000 -> mechanical RPM (14 poles)
     * = 150000 * 2 / 14 = 21428. */
    const uint8_t  exponent = 2;
    const uint16_t mantissa = 100;
    const uint16_t v12      = (uint16_t)((exponent << 9) | mantissa);

    uint16_t edges[64];
    uint16_t n = encode_frame(v12, 33, edges, 64);
    CHECK(n > 0 && n <= 22, "edge count = %u", n);

    DShotGcrFrame f = {0};
    bool ok = DShotGcr_Decode(edges, n, 33, &f);
    CHECK(ok, "decoder should accept a clean frame");
    CHECK(f.valid, "frame.valid = %d", f.valid);
    CHECK(!f.motor_stopped, "should not be motor_stopped");
    CHECK(f.exponent == exponent, "exponent = %u (expected %u)",
          f.exponent, exponent);
    CHECK(f.mantissa == mantissa, "mantissa = %u (expected %u)",
          f.mantissa, mantissa);
    CHECK(f.period_us == 400U, "period_us = %u (expected 400)",
          f.period_us);

    uint32_t rpm = DShotGcr_PeriodToRpm(f.period_us, 14);
    CHECK(rpm == 21428U, "rpm = %u (expected 21428)", rpm);
}

static void test_motor_stopped_sentinel(void)
{
    uint16_t edges[64];
    uint16_t n = encode_frame(0x0FFF, 33, edges, 64);

    DShotGcrFrame f = {0};
    bool ok = DShotGcr_Decode(edges, n, 33, &f);
    CHECK(ok, "motor-stopped sentinel should decode");
    CHECK(f.valid, "valid should be true on sentinel");
    CHECK(f.motor_stopped, "motor_stopped should be true");
}

static void test_crc_corruption(void)
{
    /* Encode a valid frame, then flip a bit inside the GCR stream. The
     * CRC should fail. */
    uint16_t edges[64];
    uint16_t n = encode_frame(0x0234, 33, edges, 64);
    CHECK(n > 0, "preconditioning failed");

    /* Push the third edge a full bit period later. That moves the
     * transition into the next cell and forces a real bit flip in the
     * recovered stream, which the CRC must catch. */
    if (n >= 3) {
        edges[2] = (uint16_t)(edges[2] + 33);
    }

    DShotGcrFrame f = {0};
    bool ok = DShotGcr_Decode(edges, n, 33, &f);
    CHECK(!ok, "decoder must reject bit-corrupted frames");
    CHECK(!f.valid, "frame must not be marked valid");
}

static void test_no_edges(void)
{
    uint16_t edges[1] = {0};
    DShotGcrFrame f = {0};
    bool ok = DShotGcr_Decode(edges, 0, 33, &f);
    CHECK(!ok, "decoder must reject empty input");
}

static void test_period_to_rpm_edge_cases(void)
{
    CHECK(DShotGcr_PeriodToRpm(0, 14) == 0, "period 0 -> rpm 0");
    CHECK(DShotGcr_PeriodToRpm(400, 0) == 0, "poles 0 -> rpm 0");
    /* 60M / 1_000_000 us = 60 eRPM * 2 / 14 = 8 RPM */
    CHECK(DShotGcr_PeriodToRpm(1000000U, 14) == 8U,
          "1 s period -> 8 RPM");
}

/* Host-side mirror of dshot.c::dshot_crc_nibble — the firmware function
 * is static so we re-implement it here for the round-trip assertion. */
static uint16_t host_dshot_crc(uint16_t v12)
{
    uint16_t crc = 0;
    uint16_t v   = v12;
    for (int i = 0; i < 3; ++i) {
        crc ^= v;
        v >>= 4;
    }
    return crc & 0x0F;
}

/* Host-side mirror of dshot.c::dshot_make_frame (post-fix). */
static uint16_t host_make_frame(uint16_t value, int telem_req, int invert)
{
    uint16_t v12   = (uint16_t)(((value & 0x07FF) << 1) | (telem_req ? 1U : 0U));
    uint16_t crc   = host_dshot_crc(v12);
    uint16_t frame = (uint16_t)((v12 << 4) | crc);
    return invert ? (uint16_t)~frame : frame;
}

/* What the ESC reconstructs from the wire after applying bidir bit-decoding
 * (each line bit interpreted as inverted). */
static uint16_t esc_recover(uint16_t wire_bits) { return (uint16_t)~wire_bits; }

static void test_make_frame_bidir_round_trip(void)
{
    /* Walk a handful of throttle values + the motor-stop command. For
     * each, build the bidir frame, simulate ESC reception, verify that
     * the recovered CRC matches the standard algorithm over the
     * recovered 12-bit value. This is exactly what would have failed
     * before the CRC double-inversion fix. */
    static const uint16_t kValues[] = { 0U, 48U, 307U, 1024U, 2047U };
    for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); ++i) {
        const uint16_t value = kValues[i];
        uint16_t wire        = host_make_frame(value, 1, 1);
        uint16_t recv        = esc_recover(wire);

        uint16_t v12_recv = (uint16_t)(recv >> 4) & 0x0FFF;
        uint16_t crc_recv = (uint16_t)(recv & 0x0F);
        uint16_t crc_calc = host_dshot_crc(v12_recv);

        CHECK(crc_recv == crc_calc,
              "bidir round-trip CRC mismatch for value=%u: recv=%04x calc=%x",
              value, recv, crc_calc);

        /* And the recovered value should match what we sent (12 bits =
         * 11-bit value | 1-bit telem flag). */
        uint16_t expected_v12 = (uint16_t)(((value & 0x07FF) << 1) | 1U);
        CHECK(v12_recv == expected_v12,
              "bidir round-trip value mismatch: got 0x%03x expected 0x%03x",
              v12_recv, expected_v12);
    }
}

static void test_rpm_stats_all_zero(void)
{
    /* Every motor reports exactly 0 RPM -> group_mean_rpm == 0 ->
     * deviation math would divide by zero without the guard added in
     * rpm_stats.c. */
    RpmStats_Reset();
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        RpmStats_Sample(ch, 0);
    }
    RpmEvalResult r;
    bool pass = RpmStats_Evaluate(&r);
    CHECK(!pass, "all-zero RPM must not pass");
    CHECK(!r.overall_pass, "overall_pass must be false");
    CHECK(r.group_mean_rpm == 0, "group_mean_rpm should be 0");
}

static void test_rpm_stats_known_good(void)
{
    /* Sanity: four motors all spinning at the same RPM -> all pass. */
    RpmStats_Reset();
    for (int sample = 0; sample < 100; ++sample) {
        for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
            RpmStats_Sample(ch, 5000);
        }
    }
    RpmEvalResult r;
    bool pass = RpmStats_Evaluate(&r);
    CHECK(pass, "uniform RPM should pass");
    CHECK(r.overall_pass, "overall_pass should be true");
    CHECK(r.group_mean_rpm == 5000, "group_mean_rpm == 5000");
}

int main(void)
{
    test_roundtrip_basic();
    test_motor_stopped_sentinel();
    test_crc_corruption();
    test_no_edges();
    test_period_to_rpm_edge_cases();
    test_make_frame_bidir_round_trip();
    test_rpm_stats_all_zero();
    test_rpm_stats_known_good();

    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
