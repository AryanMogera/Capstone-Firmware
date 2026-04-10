//timebase.h
#ifndef TIMEBASE_H_
#define TIMEBASE_H_

#include <stdint.h>

void timebase_init_1ms(uint32_t cpu_hz);
uint32_t millis(void);
void delay_us(uint32_t us);


#endif /* TIMEBASE_H_ */
