#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

/*
 * Bidirectional DShot300 driver for 4 synchronized channels.
 *
 * Hardware bindings (see motor_test_rig.ioc):
 *   - TIM1 CH1..CH4 on PA8..PA11, AF1.
 *   - All four channels share one timer => zero inter-channel skew.
 *   - DMA1 channels 2/3/4/5 feed CCR1/CCR2/CCR3/CCR4 from per-channel
 *     bit buffers; TIM1_UP triggers the burst.
 *
 * Telemetry is the bidirectional-DShot ("RPM telemetry") variant:
 *   - 16-bit frame: 11-bit throttle | 1-bit telemetry-request | 4-bit CRC.
 *   - Frame is transmitted **inverted** (this is what distinguishes
 *     bidirectional DShot from classic DShot).
 *   - ESC responds ~30 us later with a 21-edge GCR-encoded packet
 *     that decodes to a 12-bit eRPM-period value + 4-bit CRC. We
 *     convert period -> eRPM -> mechanical RPM using pole count.
 */

#define DSHOT_THROTTLE_MIN     48U
#define DSHOT_THROTTLE_MAX     2047U

/* Special low-throttle commands (DShot value < 48). */
#define DSHOT_CMD_MOTOR_STOP   0U
/* Beacons drive the motor windings as a piezo speaker; the motor hums
 * but does not spin. BEACON1 is the lowest pitch (~250 Hz), BEACON5 the
 * highest (~870 Hz). BLHeli requires >=6 consecutive frames before the
 * ESC acts on a command. */
#define DSHOT_CMD_BEACON1      1U
#define DSHOT_CMD_BEACON2      2U
#define DSHOT_CMD_BEACON3      3U
#define DSHOT_CMD_BEACON4      4U
#define DSHOT_CMD_BEACON5      5U

typedef struct {
    bool     valid;       /* true if a fresh telemetry frame was decoded */
    uint32_t rpm;         /* mechanical RPM (eRPM * 2 / pole_count)      */
    uint32_t erpm;        /* electrical RPM                              */
    uint32_t period_us;   /* raw eRPM period reported by the ESC         */
} DShotTelem;

/* One-shot init. Configures GPIO AF, TIM1 PWM mode, DMA streams, IRQs. */
void DShot_Init(void);

/*
 * Send the same DShot value to all 4 channels simultaneously and arm
 * the RX path for telemetry capture. Non-blocking: returns immediately;
 * telemetry is decoded asynchronously into the per-channel telem array.
 *
 *   value         : 0..2047 (DShot frame payload, pre-CRC)
 *   request_telem : true for bidirectional DShot (frame inverted, RX armed)
 */
void DShot_SendAll(uint16_t value, bool request_telem);

/*
 * Send a distinct DShot value on each channel in the same TX frame.
 * Used by the per-failed-motor indicate cadence to beep only the
 * failed channels while keeping the rest silent. The frame structure
 * is identical to DShot_SendAll's; only the payload per channel differs.
 */
void DShot_SendPerChannel(const uint16_t values[APP_NUM_MOTORS],
                          bool request_telem);

/*
 * Force all channels low (DShot_CMD_MOTOR_STOP). Call this when aborting
 * a test or returning to idle. Safe to call from an ISR.
 */
void DShot_StopAll(void);

/*
 * Retrieve the latest telemetry sample for a given motor channel (0..3).
 * Returns telem.valid == false if no fresh sample has arrived since the
 * previous call (caller resets validity on consume).
 */
DShotTelem DShot_ConsumeTelem(uint8_t channel);

#endif /* DSHOT_H */
