#include "dshot.h"
#include "dshot_gcr.h"

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx.h"

#include <string.h>

/*
 * --------------------------------------------------------------------------
 * Bidirectional DShot300 driver
 * --------------------------------------------------------------------------
 *
 * TX path
 *   Each of the 4 channels has its own DMA stream that walks a 17-entry
 *   CCR buffer (16 frame bits + 1 trailing zero so the line ends low). The
 *   streams are routed via DMA1->CSELR so each TIM1 CC event triggers one bit on
 *   its own channel. With TIM1 at 80 MHz and ARR+1 = 267 the bit-cell is
 *   ~3.34 us — i.e. DShot300.
 *
 *   Because all 4 channels share TIM1, the bit cells are inherently aligned:
 *   any inter-channel skew is bounded by DMA arbitration jitter (a couple
 *   of AHB cycles).
 *
 * RX path
 *   When the last CCR write completes (end-of-frame), we reconfigure
 *   CH1..CH4 as input capture and re-arm each DMA stream as a circular
 *   timestamp recorder. The ESC responds ~30 us after our frame ends with a
 *   21-edge GCR-encoded packet (~5.33 us / bit at 3/4 of 300 kHz).
 *
 *   The GCR decoder is non-trivial (clock recovery from edge timestamps,
 *   5b/4b GCR table, CRC). The structure is wired through end-to-end here;
 *   the inner edge-decode (dshot_rx_decode) is the bring-up surface area to
 *   verify against a logic analyzer.
 * --------------------------------------------------------------------------
 */

#define DSHOT_BITS_PER_FRAME    16U
#define DSHOT_BUF_LEN           (DSHOT_BITS_PER_FRAME + 1U) /* +1 trailing low */
#define DSHOT_RX_CAPS           32U                          /* > 21 GCR edges */

/*
 * DMA channel routing on STM32L432KC (RM0394 Table 41). The L432 uses
 * DMA1->CSELR for per-channel request multiplexing (no DMAMUX peripheral).
 * Each entry below picks request "0b0111" on the listed channel, which
 * selects the TIM1 event named in the comment.
 *
 *   s_dma_ch[0] -> TIM1_CH1  on DMA1_Channel2  (CSELR.C2S = 7)
 *   s_dma_ch[1] -> TIM1_CH2  on DMA1_Channel3  (CSELR.C3S = 7)
 *   s_dma_ch[2] -> TIM1_CH3  on DMA1_Channel6  (CSELR.C6S = 7)
 *   s_dma_ch[3] -> TIM1_CH4  on DMA1_Channel4  (CSELR.C4S = 7)
 *
 * Channel 5 is reserved for TIM1_UP and is intentionally unused here.
 * Note that CH3 lives on Channel 6, not Channel 4 — easy to get wrong.
 */
static DMA_Channel_TypeDef * const s_dma_ch[APP_NUM_MOTORS] = {
    DMA1_Channel2, DMA1_Channel3, DMA1_Channel6, DMA1_Channel4,
};

/* CSELR sub-field index (0..6) corresponding to s_dma_ch[i], i.e.
 * (channel_number - 1). C1S..C7S occupy bits [3:0]..[27:24] of CSELR in
 * 4-bit fields. */
static const uint8_t s_csel_shift[APP_NUM_MOTORS] = {
    (2 - 1) * 4U,   /* DMA1_Channel2 -> C2S, bits [7:4]   */
    (3 - 1) * 4U,   /* DMA1_Channel3 -> C3S, bits [11:8]  */
    (6 - 1) * 4U,   /* DMA1_Channel6 -> C6S, bits [23:20] */
    (4 - 1) * 4U,   /* DMA1_Channel4 -> C4S, bits [15:12] */
};
#define DSHOT_CSEL_REQUEST_TIM1     0x07U   /* request "0b0111" picks TIM1 events */
#define DSHOT_END_OF_FRAME_IRQn     DMA1_Channel4_IRQn  /* CH4's DMA channel */
#define DSHOT_END_OF_FRAME_TCIF     DMA_ISR_TCIF4
#define DSHOT_END_OF_FRAME_CTCIF    DMA_IFCR_CTCIF4
#define DSHOT_END_OF_FRAME_HANDLER  DMA1_Channel4_IRQHandler

static volatile uint32_t * const s_ccr[APP_NUM_MOTORS] = {
    &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3, &TIM1->CCR4,
};

/* Per-channel bit-cell CCR buffers. */
static uint32_t s_tx_buf[APP_NUM_MOTORS][DSHOT_BUF_LEN];
static uint16_t s_rx_buf[APP_NUM_MOTORS][DSHOT_RX_CAPS];

/* Latest decoded telemetry. Single-producer (ISR) / single-consumer
 * (app loop); the 'valid' flag is the handshake. */
static volatile DShotTelem s_telem[APP_NUM_MOTORS];

/* ------------------------------ frame helpers ----------------------------- */

static uint16_t dshot_crc_nibble(uint16_t value12)
{
    uint16_t crc = 0;
    uint16_t v   = value12;
    for (int i = 0; i < 3; ++i) {
        crc ^= v;
        v >>= 4;
    }
    return crc & 0x0F;
}

/*
 * Build the 16-bit DShot frame.
 *
 *   bits 15..5 : 11-bit value
 *   bit  4     : telemetry request
 *   bits 3..0  : CRC (over value<<1 | telem)
 *
 * For bidirectional DShot the entire 16-bit packet is inverted before
 * transmission; the receiver inverts back and verifies the standard CRC.
 * NOTE: do not also pre-invert the CRC nibble — that would cancel the
 * whole-frame inversion in the low nibble and the ESC would reject every
 * frame. (This was a real bug; see git history.)
 */
static uint16_t dshot_make_frame(uint16_t value, bool telem_req, bool invert)
{
    uint16_t v12   = ((value & 0x07FF) << 1) | (telem_req ? 1U : 0U);
    uint16_t crc   = dshot_crc_nibble(v12);
    uint16_t frame = (v12 << 4) | crc;
    return invert ? (uint16_t)~frame : frame;
}

static void dshot_expand_frame(uint32_t *buf, uint16_t frame)
{
    for (int i = 0; i < DSHOT_BITS_PER_FRAME; ++i) {
        buf[i] = (frame & 0x8000) ? APP_DSHOT_T1H : APP_DSHOT_T0H;
        frame <<= 1;
    }
    buf[DSHOT_BITS_PER_FRAME] = 0U;
}

/* --------------------------------- init ----------------------------------- */

static void dshot_gpio_init_af(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    LL_GPIO_InitTypeDef io = {
        .Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_UP,
        .Alternate  = LL_GPIO_AF_1,
    };
    LL_GPIO_Init(GPIOA, &io);
}

static void dshot_tim1_init(void)
{
    LL_TIM_InitTypeDef tim = {
        .Prescaler         = 0,
        .CounterMode       = LL_TIM_COUNTERMODE_UP,
        .Autoreload        = APP_DSHOT_ARR,
        .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0,
    };
    LL_TIM_Init(TIM1, &tim);

    LL_TIM_OC_InitTypeDef oc = {
        .OCMode       = LL_TIM_OCMODE_PWM1,
        .OCState      = LL_TIM_OCSTATE_ENABLE,
        .OCNState     = LL_TIM_OCSTATE_DISABLE,
        .CompareValue = 0,
        .OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        .OCIdleState  = LL_TIM_OCIDLESTATE_LOW,
    };
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &oc);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);

    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableARRPreload(TIM1);

    /* DMA on each CC event. */
    LL_TIM_EnableDMAReq_CC1(TIM1);
    LL_TIM_EnableDMAReq_CC2(TIM1);
    LL_TIM_EnableDMAReq_CC3(TIM1);
    LL_TIM_EnableDMAReq_CC4(TIM1);
}

static void dshot_csel_route(void)
{
    /* One-time CSELR setup: route each of our channels to its TIM1 CCx
     * request. C{n}S occupies a 4-bit field at bit position (n-1)*4. */
    uint32_t cselr = DMA1_CSELR->CSELR;
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        const uint32_t mask = 0xFU << s_csel_shift[ch];
        cselr = (cselr & ~mask)
              | ((uint32_t)DSHOT_CSEL_REQUEST_TIM1 << s_csel_shift[ch]);
    }
    DMA1_CSELR->CSELR = cselr;
}

static void dshot_tx_arm_dma(uint8_t ch)
{
    DMA_Channel_TypeDef *dma = s_dma_ch[ch];

    /* Disable + reconfigure. CNDTR / CPAR / CMAR are only writable while
     * the channel is disabled. */
    dma->CCR   = 0;
    dma->CNDTR = DSHOT_BUF_LEN;
    dma->CPAR  = (uint32_t)s_ccr[ch];
    dma->CMAR  = (uint32_t)s_tx_buf[ch];

    /* Mem->Periph, word transfers, mem-increment. The DShot CH4 DMA
     * channel (DMA1_Channel4 on L432) carries our "frame sent, switch
     * to RX" interrupt; only that channel enables TCIE. */
    uint32_t ccr = DMA_CCR_DIR
                 | DMA_CCR_MINC
                 | (2U << DMA_CCR_PSIZE_Pos)
                 | (2U << DMA_CCR_MSIZE_Pos)
                 | DMA_CCR_PL_1;
    if (ch == APP_NUM_MOTORS - 1) {
        ccr |= DMA_CCR_TCIE;
    }

    dma->CCR = ccr | DMA_CCR_EN;
}

void DShot_Init(void)
{
    memset((void *)s_telem, 0, sizeof(s_telem));
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    dshot_gpio_init_af();
    dshot_tim1_init();
    dshot_csel_route();

    NVIC_SetPriority(DSHOT_END_OF_FRAME_IRQn, 1);
    NVIC_EnableIRQ(DSHOT_END_OF_FRAME_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    /* Park the line low. */
    DShot_StopAll();
}

/* ------------------------------ public API -------------------------------- */

static void dshot_kick(void)
{
    /* Realign all 4 channels on the next update event before kicking. */
    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_EnableCounter(TIM1);
}

void DShot_SendAll(uint16_t value, bool request_telem)
{
    /* All four channels carry the same value — build the frame once. */
    uint16_t frame = dshot_make_frame(value, request_telem, request_telem);
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        dshot_expand_frame(s_tx_buf[ch], frame);
        dshot_tx_arm_dma(ch);
    }
    dshot_kick();
}

void DShot_SendPerChannel(const uint16_t values[APP_NUM_MOTORS], bool request_telem)
{
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        uint16_t frame = dshot_make_frame(values[ch], request_telem, request_telem);
        dshot_expand_frame(s_tx_buf[ch], frame);
        dshot_tx_arm_dma(ch);
    }
    dshot_kick();
}

void DShot_StopAll(void)
{
    /* Use bidir encoding so the ESC, which is in bidir mode, accepts the
     * command cleanly. The 'request_telem' flag also makes the frame
     * structurally identical to the throttle frames the ESC just saw,
     * which avoids it momentarily re-detecting the protocol. */
    DShot_SendAll(DSHOT_CMD_MOTOR_STOP, true);
}

DShotTelem DShot_ConsumeTelem(uint8_t channel)
{
    DShotTelem out = { .valid = false };
    if (channel >= APP_NUM_MOTORS || !s_telem[channel].valid) {
        return out;
    }

    __disable_irq();
    out                    = (DShotTelem)s_telem[channel];
    s_telem[channel].valid = false;
    __enable_irq();

    return out;
}

/* --------------------------- internal: RX path ---------------------------- */

static void dshot_rx_switch_to_input(void)
{
    /* GPIO mode + pull-up are set once in dshot_gpio_init_af and stay valid
     * for both TX (PWM out via TIM1 AF1) and RX (TIM1 input capture via
     * the same AF1): the timer owns the pin in both directions. We only
     * need to flip the timer/DMA config here. */
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        s_dma_ch[ch]->CCR = 0;
    }

    /* Drop the CC1..CC4 channel enables, then reconfigure as input
     * capture (CCxS = 01 -> ICx mapped to TIx), both edges (CCxNP=1,
     * CCxP=1), with a light 4-cycle input filter to suppress glitches. */
    TIM1->CCER  = 0;
    TIM1->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) | (2U << TIM_CCMR1_IC1F_Pos)
                | (1U << TIM_CCMR1_CC2S_Pos) | (2U << TIM_CCMR1_IC2F_Pos);
    TIM1->CCMR2 = (1U << TIM_CCMR2_CC3S_Pos) | (2U << TIM_CCMR2_IC3F_Pos)
                | (1U << TIM_CCMR2_CC4S_Pos) | (2U << TIM_CCMR2_IC4F_Pos);
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP
                | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP
                | TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP
                | TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP;

    TIM1->PSC = APP_DSHOT_RX_PSC;
    TIM1->ARR = APP_DSHOT_RX_ARR;
    TIM1->CNT = 0;
    TIM1->EGR = TIM_EGR_UG;            /* latch PSC/ARR */
    TIM1->SR  = 0;                     /* clear any pending flags */

    /* Per-channel DMA in peripheral-to-memory mode, recording CCRx into
     * s_rx_buf[ch]. 16-bit transfers — the upper half of CCRx is zero
     * for non-32-bit timers, which matches the uint16_t buffer width. */
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        DMA_Channel_TypeDef *dma = s_dma_ch[ch];
        dma->CCR   = 0;
        dma->CNDTR = DSHOT_RX_CAPS;
        dma->CPAR  = (uint32_t)s_ccr[ch];
        dma->CMAR  = (uint32_t)s_rx_buf[ch];
        /* CSELR routing was set once in DShot_Init and stays valid in
         * RX direction too — the same TIM1 CCx events drive captures. */
        dma->CCR   = DMA_CCR_MINC                       /* mem-increment */
                   | (1U << DMA_CCR_PSIZE_Pos)          /* 16-bit periph */
                   | (1U << DMA_CCR_MSIZE_Pos)          /* 16-bit mem    */
                   | DMA_CCR_PL_1;                      /* priority high */
        dma->CCR  |= DMA_CCR_EN;
    }

    /* Update IRQ acts as the RX timeout; on ARR wrap we evaluate whatever
     * captures landed. */
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void dshot_rx_disarm(void)
{
    TIM1->CR1  &= ~TIM_CR1_CEN;
    TIM1->DIER &= ~TIM_DIER_UIE;
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        s_dma_ch[ch]->CCR &= ~DMA_CCR_EN;
    }
}

static void dshot_restore_tx(void)
{
    /* Roll back to TX configuration so the next DShot_SendAll() works
     * without further reconfig. The TX init in dshot_tim1_init covers
     * PWM mode + preload; we only need to undo what RX changed. */
    TIM1->PSC   = 0;
    TIM1->ARR   = APP_DSHOT_ARR;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->DIER  = 0;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE
                | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
}

static void dshot_decode_rx(void)
{
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        /* CNDTR tells us how many entries are *unused*. The number of
         * edges we actually captured is the buffer size minus that. */
        uint16_t remaining = (uint16_t)s_dma_ch[ch]->CNDTR;
        uint16_t n_edges   = (remaining > DSHOT_RX_CAPS)
                           ? 0U
                           : (uint16_t)(DSHOT_RX_CAPS - remaining);

        DShotGcrFrame gcr;
        if (!DShotGcr_Decode(s_rx_buf[ch], n_edges,
                             APP_DSHOT_RX_BIT_TICKS, &gcr) ||
            gcr.motor_stopped) {
            /* No CRC-valid frame OR ESC reports the motor is not spinning.
             * Either way the app should treat the channel as failed per
             * PRD §5. We deliberately do NOT inject a valid rpm=0 sample
             * here: that would make group_mean_rpm 0 and divide-by-zero
             * the deviation check in RpmStats_Evaluate. */
            s_telem[ch].valid = false;
            continue;
        }

        DShotTelem t = {
            .valid     = true,
            .period_us = gcr.period_us,
            .erpm      = 60000000U / gcr.period_us,
            .rpm       = DShotGcr_PeriodToRpm(gcr.period_us,
                                              APP_MOTOR_POLE_COUNT),
        };
        s_telem[ch] = t;
    }
}

/* End-of-frame: the DMA channel that feeds TIM1_CH4 fires transfer-
 * complete after the last CCR write. On L432 that's DMA1_Channel4. */
void DSHOT_END_OF_FRAME_HANDLER(void)
{
    if (DMA1->ISR & DSHOT_END_OF_FRAME_TCIF) {
        DMA1->IFCR = DSHOT_END_OF_FRAME_CTCIF;

        /* Stop the bit clock; no more CC DMA requests on TX. */
        TIM1->CR1 &= ~TIM_CR1_CEN;

        dshot_rx_switch_to_input();
    }
}

/* RX timeout: TIM1 wrapped without a follow-up TX kick, meaning either
 * the GCR response is done or never arrived. Decode whatever the DMA
 * captured. */
void TIM1_UP_TIM16_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR = (uint32_t)~TIM_SR_UIF;

        dshot_rx_disarm();
        dshot_decode_rx();
        dshot_restore_tx();
    }
}
