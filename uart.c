#include <STC32G.H>
#include "uart.h"
#include "main.h"
#include <intrins.h>
#include <stdio.h>
#include <stdarg.h>

unsigned char uart2_rx_buf[UART2_BUF_SIZE];
unsigned char uart2_rx_head = 0;
unsigned char uart2_rx_tail = 0;

unsigned char uart2_tx_buf[UART2_BUF_SIZE];
unsigned char uart2_tx_head = 0;
unsigned char uart2_tx_tail = 0;
bit           uart2_tx_busy = 0;




void UART1_Init(void)
{
    // ��Ϊ P3.6/P3.7
    P3M0 &= ~0xC0;           //1100 0000 - 0011 1111  
    P3M1 &= ~0xC0;
    // P3.6, P3.7 ׼˫���

    // ���ܽ��л�������1�л��� P3.6(RXD1)/P3.7(TXD1)
    // P_SW1[7:6] = 01
    P_SW1 &= ~0xC0;            // ����bit7:6
    P_SW1 |=  0x40;            // ��Ϊ 01

    // ����1���ƼĴ�����ģʽ1��10λUART������������
    SCON = 0x50;                // SM0=0, SM1=1��ģʽ1��, REN=1

    // ѡ��ʱ��1��Ϊ����1�����ʷ�����
    // AUXR.6=1����ʱ��1������1Tģʽ������12T��
    // AUXR.0=0������1ʱ��Դѡ��ʱ��1
    AUXR |= 0x40;               // ��ʱ��1 1Tģʽ
    AUXR &= ~0x01;              // ����1ѡT1Ϊ�����ʷ�����

    TMOD &= 0x0F;               // ���T1ģʽλ


    // ��������ֵ
    TH1 = UART1_RELOAD >> 8;
    TL1 = UART1_RELOAD & 0xFF;

    // ������ʱ��1
    TR1 = 1;

    // ʹ�ܴ���1�ж�
    ES = 1;
    EA = 1;
}



void UART2_Init(void)
{
    // 配置 P4.6 为 RXD2（输入），P4.7 为 TXD2（推挽输出）
    P4M0 |=  0x80;              // P4.7 推挽输出
    P4M1 &= ~0x80;
    P4M0 &= ~0x40;              // P4.6 准双向（输入）
    P4M1 &= ~0x40;

    // 串口2控制寄存器（S2CON）
    // S2SM0=0, S2SM1=1：8位UART模式
    // S2REN=1：允许接收
    S2CON = 0x50;

    // 使能扩展SFR访问，配置定时器2
    P_SW2 |= 0x80;

    // 定时器2，16位自动重装，1T模式（AUXR.2=1）
    AUXR |= 0x04;
    AUXR |= 0x14;               // T2x12=1(1T模式), T2R=1(启动T2)

    // 配置T2重载值（T2L/T2H 属扩展SFR）
    T2L = UART2_RELOAD & 0xFF;
    T2H = UART2_RELOAD >> 8;

    // 串口2引脚切换：RxD2→P4.6, TxD2→P4.7
    // 使用位操作，避免误清 EAXFR 等其他位
    S2_S = 1;

    P_SW2 &= ~0x80;

    // 使能串口2中断
    IE2 |= 0x01;                // ES2=1
    EA = 1;
}


void UART1_SendByte(unsigned char dat)
{
    SBUF = dat;
    while (!TI);                // 等待发送完成
    TI = 0;
}

void UART1_SendStr(unsigned char *str)
{
    while (*str)
        UART1_SendByte(*str++);
}

void UART2_SendByte(unsigned char dat)
{
    /* 等待缓冲区有空位 */
    unsigned char next = (uart2_tx_head + 1) % UART2_BUF_SIZE;
    while (next == uart2_tx_tail);      // 缓冲区满则等待

    uart2_tx_buf[uart2_tx_head] = dat;
    uart2_tx_head = next;

    /* 若发送空闲则立即启动 */
    if (!uart2_tx_busy)
    {
        uart2_tx_busy = 1;
        S2BUF = uart2_tx_buf[uart2_tx_tail];
        uart2_tx_tail = (uart2_tx_tail + 1) % UART2_BUF_SIZE;
    }
}

void UART2_SendString(unsigned char *str)
{
    while (*str)
        UART2_SendByte(*str++);
}

/*------------------------------------------------------------------------------
 * 读取接收缓冲区（返回0表示缓冲区空）
 *----------------------------------------------------------------------------*/
bit UART2_ReadByte(unsigned char *dat)
{
    if (uart2_rx_head == uart2_rx_tail)
        return 0;                       // 缓冲区空

    *dat = uart2_rx_buf[uart2_rx_tail];
    uart2_rx_tail = (uart2_rx_tail + 1) % UART2_BUF_SIZE;
    return 1;
}


void Printf(const char *fmt, ...)
{
    char buf[100];  // 缓冲区，根据需要调整大小
    va_list args;
    int len;
    
    va_start(args, fmt);
    len = vsprintf(buf, fmt, args);
    va_end(args);
    
    UART1_SendStr((unsigned char *)buf);
}


