#include "nvconfig.h"

#include <string.h>

#ifndef NVCONFIG_HOST_TEST
#include "stm32l4xx.h"
#endif

/* --------------------------------------------------------------------------
 * Pure functions (host-testable)
 * -------------------------------------------------------------------------- */

uint32_t NvConfig_CRC32(const void *data, uint32_t len)
{
    /* Bit-by-bit CRC-32 (poly 0xEDB88320). No table — 8 KB ROM saved
     * for ~5 µs/byte at 80 MHz. We CRC at most 16 bytes per save, so
     * the cost is negligible. */
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; ++b) {
            uint32_t mask = (uint32_t)0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

size_t NvConfig_Serialize(const NvConfig *cfg, void *buf, size_t buf_len)
{
    if (cfg == NULL || buf == NULL || buf_len < NVCONFIG_BLOB_SIZE) {
        return 0;
    }
    memcpy(buf, cfg, NVCONFIG_BLOB_SIZE);
    return NVCONFIG_BLOB_SIZE;
}

void NvConfig_FillCrc(NvConfig *cfg)
{
    if (cfg == NULL) return;
    cfg->crc32 = NvConfig_CRC32(cfg, NVCONFIG_CRC_COVER_BYTES);
}

bool NvConfig_ParseBuffer(const void *buf, size_t buf_len, NvConfig *out)
{
    if (buf == NULL || out == NULL || buf_len < NVCONFIG_BLOB_SIZE) {
        return false;
    }
    NvConfig tmp;
    memcpy(&tmp, buf, NVCONFIG_BLOB_SIZE);

    if (tmp.magic != NVCONFIG_MAGIC) {
        return false;
    }
    if (tmp.version != NVCONFIG_SCHEMA_VERSION) {
        return false;
    }
    const uint32_t expect = NvConfig_CRC32(&tmp, NVCONFIG_CRC_COVER_BYTES);
    if (tmp.crc32 != expect) {
        return false;
    }
    *out = tmp;
    return true;
}

/* --------------------------------------------------------------------------
 * Hardware-bound (MCU only)
 * -------------------------------------------------------------------------- */

#ifndef NVCONFIG_HOST_TEST

#define NVCONFIG_PAGE_NUM         127U
#define NVCONFIG_PAGE_ADDR        (FLASH_BASE + (NVCONFIG_PAGE_NUM * FLASH_PAGE_SIZE))
#define FLASH_KEY1                0x45670123U
#define FLASH_KEY2                0xCDEF89ABU

static void flash_wait_ready(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { /* spin */ }
}

static bool flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) == 0U) {
        return true;        /* already unlocked */
    }
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_clear_errors(void)
{
    /* Write 1 to clear sticky error flags. */
    FLASH->SR = FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR
              | FLASH_SR_PGAERR | FLASH_SR_SIZERR  | FLASH_SR_PGSERR
              | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR
              | FLASH_SR_EOP;
}

static bool flash_erase_page_127(void)
{
    flash_wait_ready();
    flash_clear_errors();

    FLASH->CR &= ~FLASH_CR_PNB;
    FLASH->CR |= (NVCONFIG_PAGE_NUM << FLASH_CR_PNB_Pos) | FLASH_CR_PER;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait_ready();

    const uint32_t errors = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR
                                       | FLASH_SR_WRPERR | FLASH_SR_PGAERR
                                       | FLASH_SR_SIZERR | FLASH_SR_PGSERR
                                       | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
    return errors == 0U;
}

static bool flash_program_doubleword(uint32_t addr, uint64_t value)
{
    flash_wait_ready();
    flash_clear_errors();

    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint32_t *)(addr)     = (uint32_t)(value & 0xFFFFFFFFU);
    *(volatile uint32_t *)(addr + 4) = (uint32_t)(value >> 32);
    flash_wait_ready();

    const uint32_t errors = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR
                                       | FLASH_SR_WRPERR | FLASH_SR_PGAERR
                                       | FLASH_SR_SIZERR | FLASH_SR_PGSERR
                                       | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR &= ~FLASH_CR_PG;
    return errors == 0U;
}

bool NvConfig_Load(NvConfig *out)
{
    return NvConfig_ParseBuffer((const void *)NVCONFIG_PAGE_ADDR,
                                NVCONFIG_BLOB_SIZE, out);
}

bool NvConfig_Save(const NvConfig *cfg)
{
    if (cfg == NULL) return false;

    /* Pack into an 8-byte-aligned buffer. NVCONFIG_BLOB_SIZE is 24,
     * exactly 3 doublewords. */
    uint8_t buf[NVCONFIG_BLOB_SIZE] __attribute__((aligned(8)));
    if (NvConfig_Serialize(cfg, buf, sizeof(buf)) != NVCONFIG_BLOB_SIZE) {
        return false;
    }

    bool ok = true;
    __disable_irq();

    if (!flash_unlock()) {
        ok = false;
        goto out;
    }
    if (!flash_erase_page_127()) {
        ok = false;
        goto lock_out;
    }
    for (uint32_t off = 0; off < NVCONFIG_BLOB_SIZE; off += 8U) {
        uint64_t dw;
        memcpy(&dw, &buf[off], sizeof(dw));
        if (!flash_program_doubleword(NVCONFIG_PAGE_ADDR + off, dw)) {
            ok = false;
            break;
        }
    }

lock_out:
    flash_lock();
out:
    __enable_irq();
    return ok;
}

#else  /* NVCONFIG_HOST_TEST */

/* Host stubs — the test harness exercises only the pure functions. */
bool NvConfig_Load(NvConfig *out) { (void)out; return false; }
bool NvConfig_Save(const NvConfig *cfg) { (void)cfg; return false; }

#endif /* NVCONFIG_HOST_TEST */
