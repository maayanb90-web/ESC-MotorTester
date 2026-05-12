#include "trace.h"

#include "app_config.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_usart.h"

#include <string.h>

/*
 * USART2 sits on the APB1 bus (PCLK1 = 80 MHz with our clock tree).
 * 115200 baud, 8 data bits, 1 stop bit, no parity, no flow control.
 * PA2 = TX (AF7), PA15 = RX (AF3). RX is enabled but unused today —
 * cheap to leave on so a future command shell can plug in.
 */
#define TRACE_BAUD          115200U
#define TRACE_PCLK_HZ       80000000U

static void trace_gpio_init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /* PA2 = USART2_TX (AF7). PA15 = USART2_RX (AF3). */
    LL_GPIO_InitTypeDef io = {
        .Pin        = LL_GPIO_PIN_2,
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
        .Alternate  = LL_GPIO_AF_7,
    };
    LL_GPIO_Init(GPIOA, &io);

    io.Pin       = LL_GPIO_PIN_15;
    io.Alternate = LL_GPIO_AF_3;
    LL_GPIO_Init(GPIOA, &io);
}

static void trace_usart_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_USART_InitTypeDef u = {
        .BaudRate            = TRACE_BAUD,
        .DataWidth           = LL_USART_DATAWIDTH_8B,
        .StopBits            = LL_USART_STOPBITS_1,
        .Parity              = LL_USART_PARITY_NONE,
        .TransferDirection   = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling        = LL_USART_OVERSAMPLING_16,
    };
    LL_USART_Init(USART2, &u);
    LL_USART_ConfigAsyncMode(USART2);
    LL_USART_Enable(USART2);

    /* Wait until USART is ready for transmission. */
    while (!LL_USART_IsActiveFlag_TEACK(USART2)) { }
}

static void trace_write_byte(uint8_t b)
{
    while (!LL_USART_IsActiveFlag_TXE(USART2)) { }
    LL_USART_TransmitData8(USART2, b);
}

static void trace_write(const char *s)
{
    while (*s) {
        trace_write_byte((uint8_t)*s++);
    }
}

/* Decimal formatter for unsigned 32-bit values. Writes into the supplied
 * buffer (caller must guarantee >= 11 bytes). Returns the number of
 * characters written, NOT including a trailing null. The result is
 * decimal, no leading zeros, no separators. */
static uint8_t fmt_u32(char *buf, uint32_t value)
{
    char tmp[10];
    uint8_t n = 0;

    if (value == 0U) {
        buf[0] = '0';
        return 1U;
    }
    while (value != 0U) {
        tmp[n++]  = (char)('0' + (value % 10U));
        value   /= 10U;
    }
    /* Reverse into the output buffer. */
    for (uint8_t i = 0; i < n; ++i) {
        buf[i] = tmp[n - 1U - i];
    }
    return n;
}

static void trace_write_u32(uint32_t value)
{
    char buf[11];
    uint8_t n = fmt_u32(buf, value);
    for (uint8_t i = 0; i < n; ++i) {
        trace_write_byte((uint8_t)buf[i]);
    }
}

static void trace_write_bool(bool b)
{
    trace_write_byte(b ? '1' : '0');
}

/* ------------------------------ public API --------------------------------- */

void Trace_Init(void)
{
    trace_gpio_init();
    trace_usart_init();
}

void Trace_PrintHeader(void)
{
    trace_write(
        "cycle_id,t_ms,overall_pass,aborted,"
        "m0_pass,m0_mean_a,m0_mean_b,m0_mean_c,m0_half_life_ms,"
        "m1_pass,m1_mean_a,m1_mean_b,m1_mean_c,m1_half_life_ms,"
        "m2_pass,m2_mean_a,m2_mean_b,m2_mean_c,m2_half_life_ms,"
        "m3_pass,m3_mean_a,m3_mean_b,m3_mean_c,m3_half_life_ms\r\n"
    );
}

void Trace_PrintResult(const CompositeResult *r,
                       uint32_t cycle_id,
                       uint32_t t_ms,
                       bool     aborted)
{
    /* Fixed prefix: cycle_id, t_ms, overall_pass, aborted. */
    trace_write_u32(cycle_id);
    trace_write_byte(',');
    trace_write_u32(t_ms);
    trace_write_byte(',');
    trace_write_bool(r != NULL && r->overall_pass && !aborted);
    trace_write_byte(',');
    trace_write_bool(aborted);

    /* Per-motor blocks. On abort, emit zeros for the per-motor columns. */
    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        trace_write_byte(',');

        const bool     pass        = (r != NULL) && !aborted && r->per_motor_pass[i];
        const uint32_t mean_a      = (r != NULL && !aborted) ? r->plateau[0].per_motor[i].mean_rpm  : 0U;
        const uint32_t mean_b      = (r != NULL && !aborted) ? r->plateau[1].per_motor[i].mean_rpm  : 0U;
        const uint32_t mean_c      = (r != NULL && !aborted) ? r->plateau[2].per_motor[i].mean_rpm  : 0U;
        const uint32_t half_life   = (r != NULL && !aborted) ? r->spin_down.per_motor[i].mean_rpm   : 0U;

        trace_write_bool(pass);
        trace_write_byte(',');
        trace_write_u32(mean_a);
        trace_write_byte(',');
        trace_write_u32(mean_b);
        trace_write_byte(',');
        trace_write_u32(mean_c);
        trace_write_byte(',');
        trace_write_u32(half_life);
    }

    trace_write("\r\n");
}

static void trace_write_kv(const char *prefix, uint32_t value)
{
    trace_write(prefix);
    trace_write_u32(value);
}

void Trace_PrintConfigLoaded(const uint16_t tol[4], const char *src)
{
    if (tol == NULL || src == NULL) return;
    trace_write("# CONFIG source=");
    trace_write(src);
    trace_write_kv("  a:",  tol[0]);
    trace_write_kv(" b:",   tol[1]);
    trace_write_kv(" c:",   tol[2]);
    trace_write_kv(" hl:",  tol[3]);
    trace_write("\r\n");
}

void Trace_PrintConfigSaved(bool ok, uint32_t rc)
{
    if (ok) {
        trace_write("# SAVED tolerances to flash\r\n");
    } else {
        trace_write_kv("# SAVE FAILED rc=", rc);
        trace_write("\r\n");
    }
}

void Trace_PrintPost(const uint16_t valid_frames[APP_NUM_MOTORS],
                     bool overall_pass)
{
    /* `#`-prefixed comment line so CSV parsers ignore it. Operator can
     * read the per-channel valid-frame count at a glance — a c2:0
     * (etc.) jumps out as the failed channel. */
    trace_write_kv("# POST valid_frames = c0:", valid_frames[0]);
    trace_write_kv(" c1:", valid_frames[1]);
    trace_write_kv(" c2:", valid_frames[2]);
    trace_write_kv(" c3:", valid_frames[3]);
    trace_write_kv("  overall_pass=", overall_pass ? 1U : 0U);
    trace_write("\r\n");
}

void Trace_PrintCalibration(const CalibrationSummary *s)
{
    if (s == NULL) return;

    /* Line 1: human-readable. Operator skims it to sanity-check the
     * bench (does sigma look plausible?). */
    trace_write_kv("# CALIBRATION n=", s->n_cycles);
    trace_write_kv(" samples/phase=",  s->n_samples_per_phase);
    trace_write_kv("  sigma_x10 = a:", s->sigma_pct_x10[CALIBRATION_PHASE_A]);
    trace_write_kv(" b:",              s->sigma_pct_x10[CALIBRATION_PHASE_B]);
    trace_write_kv(" c:",              s->sigma_pct_x10[CALIBRATION_PHASE_C]);
    trace_write_kv(" hl:",             s->sigma_pct_x10[CALIBRATION_PHASE_HALF_LIFE]);
    trace_write_kv("  recommend_x10 = a:", s->recommended_pct_x10[CALIBRATION_PHASE_A]);
    trace_write_kv(" b:",              s->recommended_pct_x10[CALIBRATION_PHASE_B]);
    trace_write_kv(" c:",              s->recommended_pct_x10[CALIBRATION_PHASE_C]);
    trace_write_kv(" hl:",             s->recommended_pct_x10[CALIBRATION_PHASE_HALF_LIFE]);
    trace_write("\r\n");

    /* Line 2: copy-pastable into app_config.h. */
    trace_write_kv("# RECOMMEND   APP_PLATEAU_A_TOL_PCT_X10=", s->recommended_pct_x10[CALIBRATION_PHASE_A]);
    trace_write_kv("  APP_PLATEAU_B_TOL_PCT_X10=",             s->recommended_pct_x10[CALIBRATION_PHASE_B]);
    trace_write_kv("  APP_PLATEAU_C_TOL_PCT_X10=",             s->recommended_pct_x10[CALIBRATION_PHASE_C]);
    trace_write_kv("  APP_HALF_LIFE_TOL_PCT_X10=",             s->recommended_pct_x10[CALIBRATION_PHASE_HALF_LIFE]);
    trace_write("\r\n");
}
