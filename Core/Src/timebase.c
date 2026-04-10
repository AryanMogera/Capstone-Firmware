// timebase.c
#include "timebase.h"
#include "stm32f4xx.h"

static volatile uint32_t g_ms = 0;

void SysTick_Handler(void) { g_ms++; }
uint32_t millis(void) { return g_ms; }

void timebase_init_1ms(uint32_t cpu_hz) {
  SysTick->LOAD = (cpu_hz / 1000U) - 1U;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

// crude microsecond delay (ok for deadtime)
void delay_us(uint32_t us) {
  // assumes core clock is reasonably high; crude loop
  volatile uint32_t n = us * 20U;
  while (n--) { __NOP(); }
}

