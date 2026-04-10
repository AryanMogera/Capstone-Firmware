// adc_dma.h
#ifndef ADC_DMA_H_
#define ADC_DMA_H_

#include <stdint.h>

#define ADC_CH_COUNT 14u

typedef struct {
  uint16_t raw[ADC_CH_COUNT];
} adc_frame_t;

void ADC_DMA_Init(void);
void ADC_DMA_Start(void);
const adc_frame_t* ADC_GetFrame(void);

float adc_raw_to_v(uint16_t raw);


#endif /* ADC_DMA_H_ */
