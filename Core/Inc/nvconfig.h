#ifndef NVCONFIG_H
#define NVCONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Non-volatile config: persists the four calibrated per-phase
 * tolerances across power cycles. Written to the last 2 KB page of
 * the L432KC's main flash (page 127 at 0x0803F800) at the end of an
 * auto-calibration run; read back at boot by TestState_Init.
 *
 * Layout is 24 bytes (multiple of 8 for the L4 doubleword-write
 * requirement). The CRC-32 covers everything from `magic` through
 * `reserved`. Bad magic / bad CRC / unprogrammed page (all-0xFF)
 * all return false from Load → the caller falls back to the
 * compile-time defaults in app_config.h.
 */

#define NVCONFIG_MAGIC            0x4D545243U   /* 'M''T''R''C' */
#define NVCONFIG_SCHEMA_VERSION   1U

typedef struct {
    uint32_t magic;          /* offset  0 */
    uint16_t version;        /* offset  4 */
    uint16_t plat_a_x10;     /* offset  6 — same units as APP_PLATEAU_A_TOL_PCT_X10 */
    uint16_t plat_b_x10;     /* offset  8 */
    uint16_t plat_c_x10;     /* offset 10 */
    uint16_t half_life_x10;  /* offset 12 */
    uint16_t reserved;       /* offset 14 — pad to 4-byte align */
    uint32_t crc32;          /* offset 16 — CRC over bytes 0..15 */
    uint32_t pad;            /* offset 20 — pad to 8-byte (doubleword) total */
} NvConfig;                  /* sizeof == 24 */

#define NVCONFIG_BLOB_SIZE        24U
#define NVCONFIG_CRC_COVER_BYTES  16U   /* bytes 0..15: magic..reserved */

/* ---- Pure host-testable functions --------------------------------------- */

/* CRC-32, polynomial 0xEDB88320 (the IEEE 802.3 / zlib variant).
 * Initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF. */
uint32_t NvConfig_CRC32(const void *data, uint32_t len);

/* Pack `cfg` into `buf` (NVCONFIG_BLOB_SIZE bytes). Returns the
 * number of bytes written, or 0 if buf_len is too small. Does NOT
 * recompute the CRC — caller is responsible for setting cfg.crc32
 * before calling (or call NvConfig_FillCrc()). */
size_t   NvConfig_Serialize(const NvConfig *cfg, void *buf, size_t buf_len);

/* Helper: compute and store the CRC over the first 20 bytes of cfg. */
void     NvConfig_FillCrc(NvConfig *cfg);

/* Parse `buf` (must be at least NVCONFIG_BLOB_SIZE bytes) into `out`.
 * Returns true iff magic matches, version is recognised, and CRC
 * validates. Otherwise `out` is left unchanged. */
bool     NvConfig_ParseBuffer(const void *buf, size_t buf_len, NvConfig *out);

/* ---- Hardware-bound — MCU only ------------------------------------------ */

/* Read page 127 of flash and run NvConfig_ParseBuffer on it. */
bool     NvConfig_Load(NvConfig *out);

/* Erase page 127, write `cfg` to it. Returns true on success;
 * false if any flash error flag was raised. On failure, the page
 * may be left partially erased — next boot will load defaults. */
bool     NvConfig_Save(const NvConfig *cfg);

#endif /* NVCONFIG_H */
