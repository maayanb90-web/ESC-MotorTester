#ifndef TRACE_H
#define TRACE_H

#include <stdbool.h>
#include <stdint.h>

#include "rpm_stats.h"

/*
 * Production-line traceability log over USART2 (PA2/PA15) — routed
 * through the on-board ST-LINK's virtual COM port, so the same
 * micro-USB cable used to flash the Nucleo also carries the log.
 *
 * Baud 115200 / 8N1. Blocking TX (one row is ~220 chars, ~19 ms at
 * 115200 — comfortably inside the TEST_RESULT phase budget).
 *
 * Format: CSV, one header line at boot, one data row per completed
 * test cycle. Aborted cycles get a row with aborted=1.
 */

void Trace_Init(void);
void Trace_PrintHeader(void);
void Trace_PrintResult(const CompositeResult *r,
                       uint32_t cycle_id,
                       uint32_t t_ms,
                       bool     aborted);

#endif /* TRACE_H */
