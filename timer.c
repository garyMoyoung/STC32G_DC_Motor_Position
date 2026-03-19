#include <STC32G.H>
#include "main.h"

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
    // CMS[1:0]=00：边沿对齐模式
    // DIR=0：向上计数（0→ARR）
    // ARPE=1：ARR预装载使能
    PWMA_CR1 = 0x80;            // CMS=00:边沿对齐，DIR=0，ARPE=1

    // PWMx_CR2：控制寄存器2
    // MMS=000：复位，触发输出为复位信号（电机控制一般设此）
    PWMA_CR2 = 0x00;

    // 设置预分频：PSC=0 即不分频，计数时钟 = SYSCLK = 33.1776MHz
    PWMA_PSCRH = 0x00;
    PWMA_PSCRL = 0x00;

    // ARR = PWM_ARR = 3317（见 main.h，边沿对齐10kHz）
    // 实际PWM频率 = FOSC / ARR = 33177600 / 3317 = 10kHz
    PWMA_ARRH = (PWM_ARR) >> 8;
    PWMA_ARRL = (PWM_ARR) & 0xFF;

    // ---- 捕获/比较通道1 配置（互补PWM输出）----

    // CCMR1：捕获比较模式寄存器1
    // OC1M[2:0]=110：PWM模式1（计数值<CCR时输出有效电平）
    // OC1PE=1：CCR1预装载使能
    PWMA_CCMR1 = 0x68;         // OC1M=110, OC1PE=1

    // 初始占空比50%（PWM_MID = 停车中点）
    PWMA_CCR1H = PWM_MID >> 8;
    PWMA_CCR1L = PWM_MID & 0xFF;

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

/* 使能 PWM 输出引脚（ENO1P=1, ENO1N=1） */
void PWM_OutputEnable(void)
{
    P_SW2 |= 0x80;
    PWMA_ENO = 0x03;
    P_SW2 &= ~0x80;
}

/* 禁用 PWM 输出引脚（两路输出均关闭，引脚回到空闲低电平） */
void PWM_OutputDisable(void)
{
    P_SW2 |= 0x80;
    PWMA_ENO = 0x00;
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
