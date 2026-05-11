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
 *   streams are armed in DMAMUX so each TIM1 CC event triggers one bit on
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

/* DMAMUX request IDs per RM0394 Table 50. */
#define DMAMUX_REQ_TIM1_CH1     11U
#define DMAMUX_REQ_TIM1_CH2     12U
#define DMAMUX_REQ_TIM1_CH3     13U
#define DMAMUX_REQ_TIM1_CH4     14U

/* DMA1 channel mapping. We use channels 2..5; channel 1 is left free for
 * future ADC use. */
static DMA_Channel_TypeDef * const s_dma_ch[APP_NUM_MOTORS] = {
    DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5,
};

static DMAMUX_Channel_TypeDef * const s_dmamux_ch[APP_NUM_MOTORS] = {
    DMAMUX1_Channel1, DMAMUX1_Channel2, DMAMUX1_Channel3, DMAMUX1_Channel4,
};

static const uint32_t s_dmamux_req[APP_NUM_MOTORS] = {
    DMAMUX_REQ_TIM1_CH1, DMAMUX_REQ_TIM1_CH2,
    DMAMUX_REQ_TIM1_CH3, DMAMUX_REQ_TIM1_CH4,
};

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
 * For bidirectional DShot the entire frame is inverted (and the CRC is
 * computed over the inverted nibbles — equivalently, we XOR with 0x0F).
 */
static uint16_t dshot_make_frame(uint16_t value, bool telem_req, bool invert)
{
    uint16_t v12 = ((value & 0x07FF) << 1) | (telem_req ? 1U : 0U);
    uint16_t crc = dshot_crc_nibble(v12);
    if (invert) {
        crc = (~crc) & 0x0F;
    }
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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
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

static void dshot_tx_arm_dma(uint8_t ch)
{
    DMA_Channel_TypeDef *dma = s_dma_ch[ch];

    /* Disable + reconfigure. CNDTR / CPAR / CMAR are only writable while
     * the channel is disabled. */
    dma->CCR   = 0;
    dma->CNDTR = DSHOT_BUF_LEN;
    dma->CPAR  = (uint32_t)s_ccr[ch];
    dma->CMAR  = (uint32_t)s_tx_buf[ch];

    /* Mem->Periph, word transfers, mem-increment. TC IRQ only on the last
     * channel — that's our "frame sent, switch to RX" trigger. */
    uint32_t ccr = DMA_CCR_DIR
                 | DMA_CCR_MINC
                 | (2U << DMA_CCR_PSIZE_Pos)
                 | (2U << DMA_CCR_MSIZE_Pos)
                 | DMA_CCR_PL_1;
    if (ch == APP_NUM_MOTORS - 1) {
        ccr |= DMA_CCR_TCIE;
    }
    s_dmamux_ch[ch]->CCR = s_dmamux_req[ch];

    dma->CCR = ccr | DMA_CCR_EN;
}

void DShot_Init(void)
{
    memset((void *)s_telem, 0, sizeof(s_telem));
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    dshot_gpio_init_af();
    dshot_tim1_init();

    NVIC_SetPriority(DMA1_Channel5_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    /* Park the line low. */
    DShot_StopAll();
}

/* ------------------------------ public API -------------------------------- */

void DShot_SendAll(uint16_t value, bool request_telem)
{
    uint16_t frame = dshot_make_frame(value, request_telem, request_telem);

    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        dshot_expand_frame(s_tx_buf[ch], frame);
        dshot_tx_arm_dma(ch);
    }

    /* Realign all 4 channels on the next update event before kicking. */
    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_EnableCounter(TIM1);
}

void DShot_StopAll(void)
{
    DShot_SendAll(DSHOT_CMD_MOTOR_STOP, false);
}

DShotTelem DShot_ConsumeTelem(uint8_t channel)
{
    DShotTelem out = { .valid = false };
    if (channel >= APP_NUM_MOTORS) {
        return out;
    }

    __disable_irq();
    out                    = (DShotTelem)s_telem[channel];
    s_telem[channel].valid = false;
    __enable_irq();

    return out;
}

/* --------------------------- internal: RX path ---------------------------- */

/*
 * RX timing.
 *
 * TIM1 ticks at 80 MHz / (RX_PSC+1) during the RX window. We want sub-100 ns
 * resolution for clean edge timestamping while keeping ARR comfortably
 * larger than one frame (21 bits * 3.33 us ≈ 70 us). Prescaler 7 gives
 * 10 MHz (0.1 us/tick); ARR = 1500 -> 150 us window for the response,
 * which fires TIM1_UP as our RX timeout.
 *
 * Bit cell = 33.3 ticks at 10 MHz for DShot300 (3.33 us / bit). The
 * decoder is tolerant of a bit-cell of width APP_DSHOT_RX_BIT_TICKS ± 50%
 * because it samples at mid-cell.
 */
#define DSHOT_RX_PSC               7U
#define DSHOT_RX_ARR               1500U
#define APP_DSHOT_RX_BIT_TICKS     33U   /* 3.33 us @ 10 MHz */

static void dshot_rx_switch_to_input(void)
{
    /* PA8..PA11: alternate function input. Pull-up keeps line idle high
     * while the ESC's open-drain output drives transitions. */
    for (uint32_t pin = LL_GPIO_PIN_8; pin <= LL_GPIO_PIN_11;
         pin = (pin << 1)) {
        LL_GPIO_SetPinMode(GPIOA, pin, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinPull(GPIOA, pin, LL_GPIO_PULL_UP);
    }

    /* Disable each TX DMA channel before we re-arm for capture. */
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

    /* Re-prescale TIM1 for capture resolution and timeout window. */
    TIM1->PSC = DSHOT_RX_PSC;
    TIM1->ARR = DSHOT_RX_ARR;
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
        dma->CCR   = DMA_CCR_MINC                       /* mem-increment */
                   | (1U << DMA_CCR_PSIZE_Pos)          /* 16-bit periph */
                   | (1U << DMA_CCR_MSIZE_Pos)          /* 16-bit mem    */
                   | DMA_CCR_PL_1;                      /* priority high */
        s_dmamux_ch[ch]->CCR = s_dmamux_req[ch];
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
                             APP_DSHOT_RX_BIT_TICKS, &gcr)) {
            /* Leave s_telem[ch].valid = false so the app treats it as a
             * no-telemetry failure per PRD §5. */
            s_telem[ch].valid = false;
            continue;
        }

        DShotTelem t = {
            .valid     = true,
            .period_us = gcr.period_us,
            .erpm      = gcr.motor_stopped
                            ? 0U
                            : (60000000U / gcr.period_us),
            .rpm       = gcr.motor_stopped
                            ? 0U
                            : DShotGcr_PeriodToRpm(gcr.period_us,
                                                   APP_MOTOR_POLE_COUNT),
        };
        s_telem[ch] = t;
    }
}

/* End-of-frame: DMA1_Channel5 transfer-complete fires on the last CCR
 * write to CH4. Switch CH1..CH4 to input capture and let the ESC drive
 * the GCR response. */
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR = DMA_IFCR_CTCIF5;

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
