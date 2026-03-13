#include <STC32G.H>
#include <intrins.h>
#include <stdio.h>
#include <stdarg.h>
#include "pic.h"
#include "spi.h"
#include "oled.h"
#include "main.h"
#include "uart.h"
#include "timer.h"



int  g_enc_cnt;
bit           B_Change;         // 计数变化标志
volatile bit disp_flag = 0;        // OLED刷新标志
volatile unsigned int ms_tick = 0;     // 系统毫秒计数器
volatile unsigned int test_pwmb = 0;

sbit     TOG  = P0^4;        // 测试用GPIO，PWM占空比改变时翻转
void CLK_Init(void)
{
    P_SW2 |= 0x80;              // 使能扩展XFR寄存器访问

    // ��2����ʹ���ڲ����� HIRC
    HIRCCR = 0x80;              // ENHIRC=1�������ڲ�����IRC
    while (!(HIRCCR & 1));   // �ȴ� HIRCST=1������Ƶ���ȶ�

    // ��3����ѡ�� HIRC ��Ϊ��ʱ��Դ
    // MCKSEL[1:0]=00 �� �ڲ��߾���IRC
    // MCK2SEL[1:0]=00 �� ʹ�� MCKSEL ѡ���ʱ��
    CLKSEL = 0x00;              // ��ʱ�� = HIRC

    // ��4��������Ƶ��SYSCLK = MCLK = HIRC = 33.1776MHz
    CLKDIV = 0x00;              // CLKDIV=0 ��ʾ ��1��������Ƶ

    // ��5�����ر���չSFR���ʣ���ѡ�����ֿ���Ҳ���ԣ�
    // P_SW2 &= ~0x80;
}

void Sys_init(void)
{
    EAXFR = 1;
    CKCON = 0x00;
    WTST = 0x00;

    P0M1 = 0x00;   P0M0 = 0x00;
    P1M1 = 0x00;   P1M0 = 0x00;
    P2M1 = 0x00;   P2M0 = 0x00;
    P3M1 = 0x00;   P3M0 = 0x00;
    P4M1 = 0x00;   P4M0 = 0x00;
    P5M1 = 0x00;   P5M0 = 0x00;
    P6M1 = 0x00;   P6M0 = 0x00;
    P7M1 = 0x00;   P7M0 = 0x00;
    
    P2M0 |= 0x2F;   // P2.0/1/2/3/5 ������� (RST/DC/CS/MOSI/SCLK)
    P2M1 &= ~0x2F;

}




void main(void)
{
    Sys_init();
    CLK_Init();
    SPI_Init();
    UART1_Init();
    UART2_Init();
    Timer0_Init();
    PWM_Init();
    PWMB_Encoder_Init();
    OLED_Init();
    OLED_BuffClear();
    PWM_SetDuty(PWM_ARR / 4);
    OLED_BuffShowString(0, 0, "Encod:", 0);
    OLED_BuffShowString(0, 2, "Cnt:", 0);
    while (1)
    {
        UART2_SendString("UART2_Hello World!\r\n");
        UART1_SendStr("Hello World!\r\n");
        Printf("cnt: %d\r\n", g_enc_cnt);
        if (disp_flag)
        {
            disp_flag = 0;
            OLED_BuffShowNum(48, 0, g_enc_cnt, 0);
            OLED_BuffShowNum(48, 2, ms_tick, 0);
            OLED_BuffShow();
        }
    }
}


void UART1_ISR(void) interrupt 4
{
	unsigned char dat;
    if (RI)
    {
        RI = 0;
        // 此处处理接收到的调试命令
        dat = SBUF;
        // TODO: 添加调试命令解析
    }
    if (TI)
    {
        TI = 0;
    }
}

void UART2_ISR(void) interrupt 8
{
	unsigned char next;
    /* 接收中断 */
    if (S2CON & 0x01)               // S2RI
    {
        S2CON &= ~0x01;             // 清 S2RI

        next = (uart2_rx_head + 1) % UART2_BUF_SIZE;
        if (next != uart2_rx_tail)  // 缓冲区未满才存入
        {
            uart2_rx_buf[uart2_rx_head] = S2BUF;
            uart2_rx_head = next;
        }
    }

    /* 发送中断 */
    if (S2CON & 0x02)               // S2TI
    {
        S2CON &= ~0x02;             // 清 S2TI

        if (uart2_tx_head != uart2_tx_tail)     // 缓冲区还有数据
        {
            S2BUF = uart2_tx_buf[uart2_tx_tail];
            uart2_tx_tail = (uart2_tx_tail + 1) % UART2_BUF_SIZE;
        }
        else
        {
            uart2_tx_busy = 0;      // 缓冲区空，发送完毕
        }
    }
}

// void PWMB_ISR(void) interrupt 27
// {
//     TOG = ~TOG;  // 测试用GPIO翻转，观察PWM占空比变化
//     if (PWMB_SR1 & 0x02)            // CC1IF: 通道1事件
//     {
//         g_enc_cnt  = (unsigned int)PWMB_CNTRH << 8;
//         g_enc_cnt |= PWMB_CNTRL;
//         B_Change = 1;
//     }
//     PWMB_SR1 = 0x00;
//     PWMB_SR2 = 0x00;
// }


void Timer0_ISR(void) interrupt 1
{
    ms_tick++;                  // 系统时基累加

    // ---- 编码器采样（每10ms读一次）----
    if (ms_tick % 10 == 0)
    {
        g_enc_cnt = Encoder_Read();
    }

    // ---- OLED刷新（每200ms一次）----
    if (ms_tick % 20 == 0)
    {
        disp_flag = 1;
    }
    if(ms_tick >= 10000) ms_tick = 0; // 防止溢出
    // ---- MODBUS 3.5字符超时检测（9600bps约4ms）----
    // 在此处递减超时计数器，具体见MODBUS实现
}

