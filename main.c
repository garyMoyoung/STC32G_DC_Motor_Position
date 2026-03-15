/*==============================================================================
 * main.c  —  在原有代码基础上集成 Modbus RTU
 * 改动点已用 [MODBUS] 注释标出，其余保持原样
 *============================================================================*/

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
#include "rs485.h"           // [MODBUS] 新增

int  g_enc_cnt;
bit           B_Change;
volatile bit disp_flag = 0;
volatile unsigned int ms_tick = 0;
volatile unsigned int test_pwmb = 0;

sbit     TOG  = P0^4;

void CLK_Init(void)
{
    P_SW2 |= 0x80;
    HIRCCR = 0x80;
    while (!(HIRCCR & 1));
    CLKSEL = 0x00;
    CLKDIV = 0x00;
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

    P2M0 |= 0x2F;
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
    Modbus_Init();              // [MODBUS] 新增：Modbus 初始化（在 UART2_Init 之后）

    OLED_Init();
    OLED_BuffClear();
    PWM_SetDuty(PWM_ARR / 4);
    OLED_BuffShowString(0, 0, "Encod:", 0);
    OLED_BuffShowString(0, 2, "Cnt:", 0);

    /* [MODBUS] 示例：将编码器计数映射到寄存器0，可供主机读取 */
    // modbus_regs[0] = 0;  // 初始值，主循环中实时更新

    while (1)
    {
        /* [MODBUS] 轮询处理 Modbus 帧（替代原来的 UART2_SendString） */
        Modbus_Poll();

        /* [MODBUS] 将编码器计数实时写入寄存器0，主机可通过 FC03 读取 */
        modbus_regs[0] = (unsigned int)g_enc_cnt;

        /* 原有调试输出（可保留或删除） */
        // UART2_SendString("UART2_Hello World!\r\n");
        Printf("cnt: %d\n", g_enc_cnt);
        // Printf("[MB] rx_len=%d addr=%02X fc=%02X\r\n", rx_len, rx_buf[0], rx_buf[1]);
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
        S2CON &= ~0x01;

        /* [MODBUS] 直接交给 Modbus 接收状态机，不再存入通用缓冲区 */
        Modbus_RxByte(S2BUF);

        /*
         * 若你仍需保留通用 UART2 缓冲区（如调试），可同时保留下面的代码：
         *
         * next = (uart2_rx_head + 1) % UART2_BUF_SIZE;
         * if (next != uart2_rx_tail)
         * {
         *     uart2_rx_buf[uart2_rx_head] = S2BUF;
         *     uart2_rx_head = next;
         * }
         */
    }

    /* 发送中断（保持原有缓冲区发送机制不变） */
    if (S2CON & 0x02)               // S2TI
    {
        S2CON &= ~0x02;

        if (uart2_tx_head != uart2_tx_tail)
        {
            S2BUF = uart2_tx_buf[uart2_tx_tail];
            uart2_tx_tail = (uart2_tx_tail + 1) % UART2_BUF_SIZE;
        }
        else
        {
            uart2_tx_busy = 0;
        }
    }
}

void Timer0_ISR(void) interrupt 1
{
    ms_tick++;

    /* [MODBUS] 3.5字符超时检测 */
    Modbus_TimerTick();

    /* 编码器采样（每10ms） */
    if (ms_tick % 10 == 0)
    {
        g_enc_cnt = Encoder_Read();
    }

    /* OLED 刷新（每200ms） */
    if (ms_tick % 20 == 0)
    {
        disp_flag = 1;
    }

    if (ms_tick >= 10000) ms_tick = 0;
}
