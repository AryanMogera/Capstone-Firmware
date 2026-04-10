// uart_vcp.c
#include "uart_vcp.h"
#include "stm32f4xx.h"

#define RX_BUF_SZ 256
static volatile uint8_t  rx_buf[RX_BUF_SZ];
static volatile uint16_t rx_w = 0, rx_r = 0;

static inline uint16_t rb_next(uint16_t x){ return (uint16_t)((x + 1u) % RX_BUF_SZ); }

/* This function is the Inbox Manager.
   It is called every time the UART hardware receives a new byte from your computer. */
static void rb_push(uint8_t b){
  uint16_t nw = rb_next(rx_w);
  if (nw == rx_r) return; // overflow drop
  rx_buf[rx_w] = b;
  rx_w = nw;
}

void UART2_Init(uint32_t pclk1_hz, uint32_t baud) {
  // PA2 TX, PA3 RX -> AF7
  // Enable Clocks (GPIOA = Bit 0 of AHB1, USART2 = Bit 17 of APB1)
  RCC->AHB1ENR |= (1u << 0);
  RCC->APB1ENR |= (1u << 17);

  GPIOA->MODER &= ~((3u << (2 * 2)) | (3u << (3 * 2)));
  GPIOA->MODER |=  ((2u << (2 * 2)) | (2u << (3 * 2)));

  // Set Alternate Function to AF7
  GPIOA->AFR[0] &= ~((0xFu << 8) | (0xFu << 12));
  GPIOA->AFR[0] |=  ((7u   << 8) | (7u   << 12));

  // Restart USART Registers
  USART2->CR1 = 0;
  USART2->CR2 = 0;
  USART2->CR3 = 0;

  // Calculate Baud Rate (BRR)
  USART2->BRR = (pclk1_hz + (baud / 2u)) / baud;

  // Enable RX interrupt
  USART2->CR1 |= (1u << 5);

  // Enable Transmitter
  USART2->CR1 |= (1u << 3);

  // Enable Receiver
  USART2->CR1 |= (1u << 2);

  // Enable USART
  USART2->CR1 |= (1u << 13);

  // Enable Interrupt in NVIC (USART2 is Interrupt #38)
  NVIC_EnableIRQ(USART2_IRQn);
}

void UART2_WriteChar(char c) {
  while (!(USART2->SR & USART_SR_TXE)) {}
  USART2->DR = (uint8_t)c;
}

void UART2_WriteStr(const char *s) {
  while (*s) UART2_WriteChar(*s++);
}

void UART2_OnRxIRQ(void) {
  if (USART2->SR & USART_SR_RXNE) {
    uint8_t b = (uint8_t)USART2->DR;  // clears RXNE
    rb_push(b);
  }
}

void UART2_WaitTxComplete(void) {
  while (!(USART2->SR & USART_SR_TC)) {}
}

bool UART2_ReadLine(char *out, uint32_t maxlen) {
  if (maxlen < 2) return false;

  uint16_t r = rx_r;
  uint32_t n = 0;

  while (r != rx_w) {
    char c = (char)rx_buf[r];
    r = rb_next(r);

    if (c == '\r') {
      continue;   // ignore CR
    }

    if (c == '\n') {
      out[n] = '\0';
      rx_r = r;
      return (n > 0);
    }

    if (n < (maxlen - 1u)) {
      out[n++] = c;
    } else {
      // line too long: terminate, consume up to newline next time
      out[n] = '\0';
      rx_r = r;
      return true;
    }
  }

  return false;
}
