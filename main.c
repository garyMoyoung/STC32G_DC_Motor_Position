#include <STC32G.H>
#include <intrins.h>
#include <stdio.h>
#include <stdarg.h>
#include "pic.h"
#include "spi.h"
#include "oled.h"

#define  DELAY_TIME   2000
#define FOSC    33177600UL

#define UART1_BAUD      115200
#define UART1_RELOAD    (65536 - FOSC / 4 / UART1_BAUD)

#define UART2_BAUD      9600

#define UART2_RELOAD    (65536 - FOSC / 4 / UART2_BAUD)    // = 646720%
#define RS485_DIR       P12     // 方向控制引脚
#define RS485_TX        1       // 发送模式
#define RS485_RX        0       // 接收模式

#define PWM_FREQ        20000
#define PWM_ARR         (FOSC / PWM_FREQ)                   // = 1658
#define PWM_DEAD_TIME   60      // 死区时间约 60/33.1776M ≈ 1.8us

#define T0_RELOAD   (65536 - FOSC / 1000)      // = 32359

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

    // P1.2 RS485 方向控制脚，推挽输出
    P1M0 |=  0x04;
    P1M1 &= ~0x04;
    RS485_DIR = RS485_RX;       // 默认接收模式

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

void PWM_Init(void)
{
    P_SW2 |= 0x80;              // 使能扩展SFR

    // P1.0 PWM1P，P1.1 PWM1N 推挽输出        ← 改 P2→P1
    P1M0 |=  0x03;
    P1M1 &= ~0x03;

    // 引脚切换：PWMA CH1 默认 P1.0(PWM1P) / P1.1(PWM1N)
    // C1PS[1:0]=00 → 默认引脚，直接清零低4位    ← 改为 0x00
    PWMA_PS &= ~0x0F;
    // PWMA_PS |= 0x00;  // 默认值，无需赋值

    // ---- PWMA 主控寄存器 ----

    // PWMx_CR1：控制寄存器1
    // CMS[1:0]=01：中央对齐模式1（上计数时比较匹配产生中断标志）
    // DIR=0：向上计数
    // ARPE=1：ARR预装载使能
    PWMA_CR1 = 0x60;            // CMS=11:中央对齐3，ARPE=1

    // PWMx_CR2：控制寄存器2
    // MMS=000：复位，触发输出为复位信号（电机控制一般设此）
    PWMA_CR2 = 0x00;

    // 设置预分频：PSC=0 即不分频，计数时钟 = SYSCLK = 33.1776MHz
    PWMA_PSCRH = 0x00;
    PWMA_PSCRL = 0x00;

    // 设置自动重载值 ARR = PWM_ARR = 1658
    // 中央对齐模式下，实际PWM频率 = SYSCLK / (2 * ARR)
    // = 33177600 / (2 * 1658) ≈ 10KHz（中央对齐等效20KHz开关频率）
    // 若要真正20KHz：ARR = 33177600/20000/2 = 829
    #undef  PWM_ARR
    #define PWM_ARR   829       // 中央对齐模式下的ARR值

    PWMA_ARRH = (PWM_ARR) >> 8;
    PWMA_ARRL = (PWM_ARR) & 0xFF;

    // ---- 捕获/比较通道1 配置（互补PWM输出）----

    // CCMR1：捕获比较模式寄存器1
    // OC1M[2:0]=110：PWM模式1（计数值<CCR时输出有效电平）
    // OC1PE=1：CCR1预装载使能
    PWMA_CCMR1 = 0x68;         // OC1M=110, OC1PE=1

    // 初始占空比50%：CCR1 = ARR/2
    PWMA_CCR1H = (PWM_ARR / 2) >> 8;
    PWMA_CCR1L = (PWM_ARR / 2) & 0xFF;

    // CCER1：捕获比较使能寄存器
    // CC1E=1：通道1正向输出使能（PWM1P）
    // CC1P=0：正向输出高电平有效
    // CC1NE=1：通道1互补输出使能（PWM1N）
    // CC1NP=0：互补输出高电平有效
    PWMA_CCER1 = 0x05;         // CC1E=1, CC1NE=1

    // ---- 死区配置 ----
    // BKR：刹车寄存器
    // MOE=1：主输出使能（必须置1，否则PWM无输出！）
    // OSSR=1：运行模式下关闭状态选择
    // OSSI=1：空闲模式下关闭状态选择
    PWMA_BKR = 0xC0;           // MOE=1, OSSR=0, OSSI=0, 无刹车

    // DTR：死区时间寄存器
    // DTG[7:0]=60：死区时间 = 60 × Tdts，Tdts=1/SYSCLK≈30ns
    // 死区 ≈ 60 × 30ns = 1.8us（满足大多数MOS驱动芯片需求）
    PWMA_DTR = PWM_DEAD_TIME;

    // 空闲状态输出：OISR
    // OIS1=0：通道1空闲时输出低
    // OIS1N=0：互补通道空闲时输出低
    PWMA_OISR = 0x00;

    // ---- 使能PWMA计数器 ----
    // CR1.CEN=1：启动计数器
    PWMA_CR1 |= 0x01;

    // 使能通道1和互补通道1输出
    PWMA_ENO = 0x03;           // ENO1P=1, ENO1N=1

    P_SW2 &= ~0x80;
}





void PWM_SetDuty(unsigned int duty)
{
    if (duty > PWM_ARR) duty = PWM_ARR;
    P_SW2 |= 0x80;
    PWMA_CCR1H = duty >> 8;
    PWMA_CCR1L = duty & 0xFF;
    P_SW2 &= ~0x80;
}

void PWMB_Encoder_Init(void)
{
    EAXFR = 1;
    P_SW2 |= 0x80;

    PWMB_ENO    = 0x00;                     // 禁用所有输出
    PWMB_PS     = 0x00;                     // 引脚映射到默认位置 (P2.0/P2.1)
    
    // 预分频设置
    PWMB_PSCRH  = 0x00;
    PWMB_PSCRL  = 0x00;
    
    // 自动重装载值（16位，支持有符号）
    PWMB_ARRH   = 0xFF;
    PWMB_ARRL   = 0xFF;

    // 计数器初值
    PWMB_CNTRH  = 0x00;
    PWMB_CNTRL  = 0x00;

    // ===== 关键：输入捕获模式配置 =====
    // CCMR1: IC1F[7:4] = 0011(滤波), CC1S[1:0] = 01(TI1输入)
    // = 0011_0001 = 0x31
    PWMB_CCMR1  = 0x31;        // TI1输入，滤波4时钟
    
    // CCMR2: IC2F[7:4] = 0011(滤波), CC2S[1:0] = 01(TI2输入)  
    // = 0011_0001 = 0x31
    PWMB_CCMR2  = 0x31;        // TI2输入，滤波4时钟
 
    // ===== 编码器模式：双边沿计数 =====
    // SMS[2:0] = 011（编码器模式3：TI1和TI2都计数）
    PWMB_SMCR   = 0x03;
 
    // ===== 极性配置 =====
    // CC1E=1, CC1P=0, CC2E=1, CC2P=0
    // 这样可以正确检测方向
    // CCER1 = 0000_0101 = 0x05
    // CCER2 = 0000_0101 = 0x05
    PWMB_CCER1  = 0x05;        // CC1E=1, CC1P=0 (不反相)
    PWMB_CCER2  = 0x05;        // CC2E=1, CC2P=0 (不反相)
    
    // 禁用中断（轮询模式）
    PWMB_IER    = 0x00;
    
    // 启动计数器
    PWMB_CR1    = 0x81;        // CEN=1, ARPE=1

    P_SW2 &= ~0x80;
    EAXFR = 0;
}

int Encoder_Read(void)
{
    int cnt;
    unsigned int raw;
    
    P_SW2 |= 0x80;
    raw  = (unsigned int)PWMB_CNTRH << 8;
    raw |= PWMB_CNTRL;
    P_SW2 &= ~0x80;
    
    // 转换为有符号16位整数
    // 如果最高位为1，则为负数
    if (raw & 0x8000) {
        cnt = -(int)(0x10000 - raw);  // 补码转换
    } else {
        cnt = (int)raw;
    }
    
    return cnt;
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
void Timer0_Init(void)
{
    // 定时器0模式0：16位自动重载
    // TMOD低4位控制T0：M1M0=00
    TMOD &= 0xF0;               // 清T0模式位，M1=0 M0=0

    // 1T模式：AUXR.7 = T0x12 = 1
    AUXR |= 0x80;

    // 设置重载值
    TH0 = T0_RELOAD >> 8;       // 高字节 = 0x7E
    TL0 = T0_RELOAD & 0xFF;     // 低字节 = 0x67

    // 清中断标志，使能T0中断
    TF0 = 0;
    ET0 = 1;
    EA  = 1;

    // 启动定时器0
    TR0 = 1;
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

