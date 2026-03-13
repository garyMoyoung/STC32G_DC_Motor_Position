#ifndef __UART_H__
#define __UART_H__







#define UART2_BUF_SIZE  64   // ? ??? extern ??????
extern unsigned char uart2_rx_buf[UART2_BUF_SIZE];
extern unsigned char uart2_rx_head;
extern unsigned char uart2_rx_tail;

extern unsigned char uart2_tx_buf[UART2_BUF_SIZE];
extern unsigned char uart2_tx_head;
extern unsigned char uart2_tx_tail;
extern bit           uart2_tx_busy;




void UART1_Init(void);
void UART2_Init(void);
void UART1_SendByte(unsigned char dat);
void UART1_SendStr(unsigned char *str);
void UART2_SendByte(unsigned char dat);
void UART2_SendString(unsigned char *str);
bit UART2_ReadByte(unsigned char *dat);
void Printf(const char *fmt, ...);


#endif /* __UART_H__ */


