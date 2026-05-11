#include "dshot.h"

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

/* --------------------------- internal: RX hook ---------------------------- */

/*
 * Decode the GCR-encoded eRPM-period response from the captured edge
 * timestamps for one channel. Returns true on a valid frame (CRC OK).
 *
 * This is the firmware-bring-up surface: the structure is here, the actual
 * 5b/4b GCR table + clock-recovery logic must be verified against a logic
 * analyzer. Until that's done the function returns false, which the
 * application interprets as "no telemetry => motor failed" per PRD §5.
 */
static bool dshot_rx_decode(uint8_t ch, DShotTelem *out)
{
    (void)ch;
    (void)out;
    /* TODO(rx-bringup): GCR clock-recovery + 5b/4b decode + CRC check.
     * Until implemented, leave telemetry invalid so the failure surfaces
     * as a "no-telemetry" fail rather than silent pass. */
    return false;
}

/* End-of-frame: DMA1_Channel5 transfer-complete signals the last CCR
 * write on CH4. We use it to flip CH1..CH4 to input capture and arm the
 * RX path. */
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR = DMA_IFCR_CTCIF5;

        /* Stop the bit clock; no more CC DMA requests. */
        LL_TIM_DisableCounter(TIM1);

        /* TODO(rx-bringup):
         *   1) Reconfigure PA8..PA11 to AF input (OPENDRAIN OFF, pull-up keeps line idle high).
         *   2) Switch TIM1 CH1..CH4 to input-capture mode (CCMR1/CCMR2).
         *   3) Arm DMA1_Channel2..5 as circular timestamp recorders.
         *   4) Configure TIM1_UP as a ~100 us RX timeout.
         *
         * Once the captures land, dshot_rx_decode() per channel writes
         * s_telem[ch] and sets .valid = true.
         */
    }
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR = (uint32_t)~TIM_SR_UIF;

        /* RX-watchdog. If a frame didn't decode, leave .valid = false so
         * the application treats it as a failure per PRD §5 ("no telemetry
         * from that channel" => motor fails). */
        for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
            DShotTelem t = { .valid = false };
            if (dshot_rx_decode(ch, &t)) {
                s_telem[ch] = t;
            }
        }
    }
}
