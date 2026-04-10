//uart_vcp.h

#ifndef UART_VCP_H_
#define UART_VCP_H_

#include <stdint.h>
#include <stdbool.h>

void UART2_Init(uint32_t pclk1_hz, uint32_t baud);
void UART2_WriteChar(char c);
void UART2_WriteStr(const char *s);
bool UART2_ReadLine(char *out, uint32_t maxlen);
void UART2_OnRxIRQ(void);
void UART2_WaitTxComplete(void);

#endif /* UART_VCP_H_ */
