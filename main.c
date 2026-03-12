#include <STC32G.H>
#include <intrins.h>
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
    // ����P1.0ΪRXD2���룬P1.1ΪTXD2�������
    P1M0 |=  0x02;              // P1.1 �������
    P1M1 &= ~0x02;
    P1M0 &= ~0x01;              // P1.0 ׼˫�����룩
    P1M1 &= ~0x01;

    // P1.2��RS485����������ţ��������
    P1M0 |=  0x04;
    P1M1 &= ~0x04;
    RS485_DIR = RS485_RX;       // Ĭ�Ͻ���ģʽ

    // ����2���ƼĴ�����S2CON��
    // S2SM0=0, S2SM1=1��8λUARTģʽ
    // S2REN=1����������
    S2CON = 0x50;

    // ʹ����չSFR���ʣ����ö�ʱ��2
    P_SW2 |= 0x80;

    // ��ʱ��2��16λ�Զ����أ�1Tģʽ��AUXR.2=1��
    AUXR |= 0x04;               // T2_CT=0��ʱ����T2R=1����T2��T2x12=1��1T
    // ע�⣺AUXR bit2=T2x12(1T), bit4=T2R(����), ��ֿ�����
    // �ֲ᣺AUXR |= 0x14 ��ͬʱ����T2x12��T2R
    AUXR |= 0x14;               // T2x12=1(1Tģʽ), T2R=1(����T2)

    // ����T2����ֵ��T2L/T2H����չSFR����
    T2L = UART2_RELOAD & 0xFF;
    T2H = UART2_RELOAD >> 8;

    // ����2ʱ��Դѡ��Ĭ��ʹ��T2������������ã�

    P_SW2 &= ~0x80;

    // ʹ�ܴ���2�ж�
    IE2 |= 0x01;                // ES2=1��ʹ�ܴ���2�ж�
    EA = 1;
}

void PWM_Init(void)
{
    P_SW2 |= 0x80;              // 使能扩展SFR

    // P2.0 PWM1P，P2.1 PWM1N 推挽输出
    P2M0 |=  0x03;
    P2M1 &= ~0x03;

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

void main(void)
{
    Sys_init();
    CLK_Init();
    SPI_Init();
    UART1_Init();
    UART2_Init();
    OLED_Init();
    OLED_BuffClear();
    OLED_BuffShowString(0,0,"Press any key to",0);
    OLED_BuffShowString(0,2,"start fucker",0);
    OLED_BuffShow();

    while (1)
    {
        UART1_SendStr("Hello World!\r\n");
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
