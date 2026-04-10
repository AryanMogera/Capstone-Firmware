// adc_dma.c
#include "adc_dma.h"
#include "stm32f4xx.h"

#define VREF   3.3f
#define ADCMAX 4095.0f

#define GPIOAEN (1U<<0)
#define GPIOBEN (1U<<1)
#define GPIOCEN (1U<<2)

/* Clock Enable bits */
#define DMA2EN  (1U<<22)
#define ADC1EN  (1U<<8)

/* DMA Control Registers Bits */
#define DMA_EN         (1U<<0)
#define DMA_CIRC       (1U<<8)
#define DMA_MINC       (1U<<10)
#define DMA_PSIZE_16   (1U<<11)
#define DMA_MSIZE_16   (1U<<13)
#define DMA_PL_MED     (2U<<16)

/* ADC Control Register Bits */
#define ADC_CR_2_ADON      (1U << 0)
#define ADC_CR_2_CONT      (1U << 1)
#define ADC_CR_2_DMA       (1U << 8)
#define ADC_CR_2_DDS       (1U << 9)
#define ADC_CR2_SWSTART    (1U << 30)
#define ADC_CR_1_SCAN      (1U << 8)

static adc_frame_t g_adc;

// ADC channel order (DMA buffer indexes)
// Voltage 1-4, Temperature 1-8, Current 1, CHG_DET

static const uint8_t g_seq[ADC_CH_COUNT] = {
  8,   // V1:        PB0  ADC12_IN8   (lowest tap)
  4,   // V2:        PA4  ADC12_IN4
  1,   // V3:        PA1  ADC123_IN1
  0,   // V4:        PA0  ADC123_IN0  (highest tap)

  6,   // T1:        PA6  ADC12_IN6
  9,   // T2:        PB1  ADC12_IN9
  15,  // T3:        PC5  ADC12_IN15
  14,  // T4:        PC4  ADC12_IN14
  13,  // T5:        PC3  ADC123_IN13
  12,  // T6:        PC2  ADC123_IN12
  10,  // T7:        PC0  ADC123_IN10
  11,  // T8:        PC1  ADC123_IN11

  5,   // I1:        PA5  ADC12_IN5
  7    // CHG_DET:   PA7  ADC12_IN7
};

static void gpio_analog_init(void) {
  RCC->AHB1ENR |= GPIOAEN | GPIOBEN | GPIOCEN;

  // PA0, PA1, PA4, PA5, PA6, PA7 analog
  GPIOA->MODER |=  (3U<<(0*2)) | (3U<<(1*2)) | (3U<<(4*2)) |
                   (3U<<(5*2)) | (3U<<(6*2)) | (3U<<(7*2));
  GPIOA->PUPDR &= ~((3U<<(0*2)) | (3U<<(1*2)) | (3U<<(4*2)) |
                    (3U<<(5*2)) | (3U<<(6*2)) | (3U<<(7*2)));

  // PB0, PB1 analog
  GPIOB->MODER |=  (3U<<(0*2)) | (3U<<(1*2));
  GPIOB->PUPDR &= ~((3U<<(0*2)) | (3U<<(1*2)));

  // PC0..PC5 analog
  for (int p = 0; p <= 5; p++) {
    GPIOC->MODER |=  (3U << ((uint32_t)p * 2U));
    GPIOC->PUPDR &= ~(3U << ((uint32_t)p * 2U));
  }
}

static void adc_set_sampling(void) {
  const uint32_t SMP = 7U; // 480 cycles for better stability on noisy/high-impedance inputs

  // ch 0..9 => SMPR2
  for (int ch = 0; ch <= 9; ch++) {
    ADC1->SMPR2 &= ~(7U << (3U * ch));
    ADC1->SMPR2 |=  (SMP << (3U * ch));
  }

  // ch 10..15 => SMPR1
  for (int ch = 10; ch <= 15; ch++) {
    uint32_t s = 3U * (uint32_t)(ch - 10);
    ADC1->SMPR1 &= ~(7U << s);
    ADC1->SMPR1 |=  (SMP << s);
  }
}

static void adc_set_sequence(const uint8_t *seq, uint8_t n) {
  ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | ((uint32_t)(n - 1U) << 20);
  ADC1->SQR2 = 0;
  ADC1->SQR3 = 0;

  for (uint8_t i = 0; i < n; i++) {
    uint32_t ch = (uint32_t)(seq[i] & 0x1FU);
    if      (i < 6)  ADC1->SQR3 |= ch << (5U * i);
    else if (i < 12) ADC1->SQR2 |= ch << (5U * (i - 6U));
    else             ADC1->SQR1 |= ch << (5U * (i - 12U));
  }
}

static void dma2_stream0_init(uint16_t *dst, uint32_t count) {
  RCC->AHB1ENR |= DMA2EN;

  DMA2_Stream0->CR &= ~DMA_EN;
  while (DMA2_Stream0->CR & DMA_EN) {}

  DMA2->LIFCR = 0x3DU;

  DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
  DMA2_Stream0->M0AR = (uint32_t)dst;
  DMA2_Stream0->NDTR = count;

  DMA2_Stream0->CR &= ~((3U<<11) | (3U<<13) | (3U<<16));
  DMA2_Stream0->CR  = DMA_CIRC | DMA_MINC | DMA_PSIZE_16 | DMA_MSIZE_16 | DMA_PL_MED;

  DMA2_Stream0->FCR = 0;
  DMA2_Stream0->CR |= DMA_EN;
}

void ADC_DMA_Init(void) {
  gpio_analog_init();

  RCC->APB2ENR |= ADC1EN;

  ADC1->CR1 = 0;
  ADC1->CR2 = 0;

  ADC1->CR1 |= ADC_CR_1_SCAN;
  ADC1->CR2 |= ADC_CR_2_CONT;
  ADC1->CR2 |= ADC_CR_2_DMA | ADC_CR_2_DDS;

  adc_set_sampling();
  adc_set_sequence(g_seq, ADC_CH_COUNT);

  dma2_stream0_init(g_adc.raw, ADC_CH_COUNT);

  ADC1->CR2 |= ADC_CR_2_ADON;
  for (volatile int i = 0; i < 1000; i++) { __NOP(); }
}

void ADC_DMA_Start(void) {
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

const adc_frame_t* ADC_GetFrame(void) {
  return &g_adc;
}

float adc_raw_to_v(uint16_t raw) {
  return (raw * VREF) / ADCMAX;
}
