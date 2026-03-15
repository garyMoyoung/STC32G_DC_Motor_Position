/*==============================================================================
 * main.c  —  集成 Modbus RTU 协议寄存器映射
 *
 * 寄存器映射（对应通信协议 v1.0）：
 *   0x0000  Motor_Angle       电机当前角度（只读，0.1°，带符号）
 *   0x0001  Motor_Speed       电机当前转速（只读，rpm，带符号）
 *   0x0002  Encoder_Count_H   编码器累计值高16位（只读）
 *   0x0003  Encoder_Count_L   编码器累计值低16位（只读）
 *   0x0004  Set_Speed         目标转速设定（可读写，rpm）
 *   0x0005  Set_Angle         目标角度设定（可读写，0.1°）
 *   0x0006  Control_Mode      控制模式（0=速度 1=角度 2=手动）
 *   0x0007  Status_Flags      状态标志位（只读）
 *   0x0008  PWM_Duty_Raw      原始PWM占空比（手动模式可写）
 *   0x0009  ms_Tick           系统时基计数（只读）
 *   0x000A~0x000F  预留，读取恒为0
 *
 * 线圈映射：
 *   0x0000  Motor_Enable      电机使能（1=使能，0=禁用）
 *   0x0001  Motor_Dir         旋转方向（1=正转，0=反转）
 *   0x0002  Brake             制动（1=制动，0=释放）
 *   0x0003  Alarm_Clear       故障清除（写1触发，自动归0）
 *   0x0004  LED_Status        状态指示灯
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
#include "rs485.h"
#include "key.h"

/*=============================================================================
 * 编码器参数（根据硬件确定，修改此处即可）
 *
 *   ENC_LINES      编码器物理线数        = 11
 *   ENC_MULT       AB两相四倍频          = 4
 *   GEAR_RATIO     减速比（输出轴：电机） = 60
 *   ENC_PPR_MOTOR  电机轴每转脉冲数      = 11 × 4 = 44
 *   ENC_PPR_OUTPUT 输出轴每转脉冲数      = 44 × 60 = 2640
 *
 * 速度公式（采样周期 10ms）：
 *   speed(rpm) = enc_delta / ENC_PPR_OUTPUT / 0.01s * 60
 *              = enc_delta * 6000 / 2640
 *              = enc_delta * 25 / 11        （整数化，避免浮点）
 *
 * 角度公式（输出轴，0.1° 精度）：
 *   angle(0.1°) = (enc_total % ENC_PPR_OUTPUT) * 3600 / ENC_PPR_OUTPUT
 *===========================================================================*/
#define ENC_LINES           11
#define ENC_MULT            4
#define GEAR_RATIO          60
#define ENC_PPR_MOTOR       (ENC_LINES * ENC_MULT)          /* 44  脉冲/电机转 */
#define ENC_PPR_OUTPUT      (ENC_PPR_MOTOR * GEAR_RATIO)    /* 2640 脉冲/输出轴转 */
 
/*=============================================================================
 * 全局状态变量
 *===========================================================================*/
int           g_enc_cnt     = 0;    /* 编码器当前原始计数（Timer0每10ms更新） */
long          g_enc_total   = 0;    /* 编码器累计计数（32位，不自动清零）     */
int           g_motor_speed = 0;    /* 当前转速（rpm，10ms差分计算）          */
int           g_motor_angle = 0;    /* 当前角度（0.1°，累计计数换算）         */
 
int           g_set_speed   = 0;    /* 目标转速（上位机写 0x0004）            */
int           g_set_angle   = 0;    /* 目标角度（上位机写 0x0005，0.1°）      */
unsigned char g_ctrl_mode   = 0;    /* 0=速度  1=角度  2=手动                */

/*
 * Status_Flags（0x0007）位定义：
 *   bit0 = 到位标志（到达目标角度/速度）
 *   bit1 = 超载标志（预留，硬件检测置位）
 *   bit2 = 当前旋转方向（1=正转，0=反转）
 */
unsigned int  g_status_flags = 0;

bit           B_Change;
volatile bit  disp_flag  = 0;
volatile unsigned int ms_tick   = 0;
volatile unsigned int test_pwmb = 0;

static int g_enc_last = 0;          /* 速度计算：上次采样值 */

sbit TOG = P0^4;

/*=============================================================================
 * Modbus_SyncRegs
 * 主循环每轮调用一次：
 *   - 把最新运行状态刷新到只读寄存器
 *   - 把上位机写入的可写寄存器取回到控制变量
 *===========================================================================*/
static void Modbus_SyncRegs(void)
{
    unsigned char k;

    /* ── 只读寄存器：用最新状态覆盖 ── */
    modbus_regs[0] = (unsigned int)g_motor_angle;          /* Motor_Angle  */
    modbus_regs[1] = (unsigned int)g_motor_speed;          /* Motor_Speed  */
    modbus_regs[2] = (unsigned int)(g_enc_total >> 16);    /* Encoder_H    */
    modbus_regs[3] = (unsigned int)(g_enc_total & 0xFFFF); /* Encoder_L    */
    modbus_regs[7] = g_status_flags;                       /* Status_Flags */
    modbus_regs[9] = ms_tick;                              /* ms_Tick      */

    /* 预留寄存器始终为0 */
    for (k = 10; k < 16; k++)
        modbus_regs[k] = 0x0000;

    /* ── 可写寄存器：取回上位机设定值 ── */
    g_set_speed = (int)modbus_regs[4];       /* Set_Speed    */
    g_set_angle = (int)modbus_regs[5];       /* Set_Angle    */
    g_ctrl_mode = (unsigned char)modbus_regs[6]; /* Control_Mode */

    /* 线圈 Alarm_Clear（bit3）：写1后自动归0，同时清故障标志 */
    if (modbus_coils & (1 << 3))
    {
        g_status_flags = 0;
        modbus_coils  &= ~(1 << 3);
    }
}

/*=============================================================================
 * Motor_ControlUpdate
 * 根据 g_ctrl_mode 执行对应的输出逻辑
 * TODO：把速度/角度环 PID 替换到对应位置
 *===========================================================================*/
static void Motor_ControlUpdate(void)
{
    unsigned int pwm_duty;

    /* Motor_Enable 线圈（bit0）：未使能时强制停机 */
    if (!(modbus_coils & (1 << 0)))
    {
        PWM_SetDuty(0);
        return;
    }

    switch (g_ctrl_mode)
    {
        /* ── 速度模式 ── */
        case 0:
            /*
             * TODO：将 g_set_speed 送入速度环PID，输出 PWM 占空比
             * 当前为线性折算示例（1000 rpm → 满占空比）
             */
            if (g_set_speed < 0)
            {
                modbus_coils |=  (1 << 1);              /* Dir = 反转 */
                pwm_duty = (unsigned int)(-g_set_speed) * PWM_ARR / 1000;
            }
            else
            {
                modbus_coils &= ~(1 << 1);              /* Dir = 正转 */
                pwm_duty = (unsigned int)( g_set_speed) * PWM_ARR / 1000;
            }
            if (pwm_duty > PWM_ARR) pwm_duty = PWM_ARR;
            PWM_SetDuty(pwm_duty);
            break;

        /* ── 角度模式 ── */
        case 1:
            /*
             * TODO：将 g_set_angle 和 g_motor_angle 差值送入位置环PID
             * 当前只做到位判断示例
             */
            if (g_motor_angle == g_set_angle)
            {
                g_status_flags |=  (1 << 0);            /* bit0 到位 */
                PWM_SetDuty(0);
            }
            else
            {
                g_status_flags &= ~(1 << 0);
                /* TODO: PID 输出到 PWM_SetDuty() */
            }
            break;

        /* ── 手动模式：直接用 PWM_Duty_Raw（0x0008） ── */
        case 2:
            pwm_duty = modbus_regs[8];
            if (pwm_duty > PWM_ARR) pwm_duty = PWM_ARR;
            PWM_SetDuty(pwm_duty);
            break;

        default:
            PWM_SetDuty(0);
            break;
    }

    /* 同步旋转方向到 Status_Flags bit2 */
    if (modbus_coils & (1 << 1))
        g_status_flags |=  (1 << 2);
    else
        g_status_flags &= ~(1 << 2);
}

/*=============================================================================
 * CLK / Sys 初始化
 *===========================================================================*/
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
    WTST  = 0x00;

    P0M1 = 0x00;   P0M0 = 0x00;
    P1M1 = 0x00;   P1M0 = 0x00;
    P2M1 = 0x00;   P2M0 = 0x00;
    P3M1 = 0x00;   P3M0 = 0x00;
    P4M1 = 0x00;   P4M0 = 0x00;
    P5M1 = 0x00;   P5M0 = 0x00;
    P6M1 = 0x00;   P6M0 = 0x00;
    P7M1 = 0x00;   P7M0 = 0x00;

    P2M0 |=  0x2F;
    P2M1 &= ~0x2F;
}

/*=============================================================================
 * main
 *===========================================================================*/
void main(void)
{
    unsigned char k;        /* C51 所有局部变量必须在函数开头声明 */

    Sys_init();
    CLK_Init();
    SPI_Init();
    UART1_Init();
    UART2_Init();
    Timer0_Init();
    PWM_Init();
    PWMB_Encoder_Init();
    Modbus_Init();          /* 在 UART2_Init 之后调用 */
    PWM_SetDuty(0);         /* 上电先停机 */

    /* 可写寄存器初始值 */
    modbus_regs[4] = 0;     /* Set_Speed  = 0 rpm  */
    modbus_regs[5] = 0;     /* Set_Angle  = 0°     */
    modbus_regs[6] = 0;     /* Control_Mode = 速度 */
    modbus_regs[8] = 0;     /* PWM_Duty_Raw = 0    */

    /* Motor_Enable 线圈默认关闭，等上位机显式使能 */
    modbus_coils = 0x00;

    /* OLED 初始化：只写一次静态标签 */
    OLED_Init();
    OLED_BuffClear();
    OLED_BuffShow();
    OLED_BuffShowString(0, 0, "Spd:", 0);   /* 第0行显示转速 */
    OLED_BuffShowString(0, 2, "Ang:", 0);   /* 第2行显示角度 */
    OLED_BuffShow();

    while (1)
    {
        /* 1. 同步寄存器（只读刷新 + 可写取回） */
        Modbus_SyncRegs();

        /* 2. Modbus 轮询（处理已收到的完整帧） */
        Modbus_Poll();

        /* 3. 电机控制输出 */
        Motor_ControlUpdate();

        /* 4. 按键处理 */
        Key_HandleEvent();

        /* 5. OLED 刷新（每20ms由 Timer0 置 disp_flag） */
        if (disp_flag)
        {
            disp_flag = 0;
            /* 显示当前转速（rpm） */
            OLED_BuffShowNum(32, 0, (long)g_motor_speed, 0);
            /* 显示当前角度（整数°，0.1°单位除以10） */
            OLED_BuffShowNum(32, 2, (long)(g_motor_angle / 10), 0);

            OLED_BuffShow();
        }
        {
            Printf("Angle: Speed: Enc: flag: %d.%d°, %d, %ld, %d\n",
                   g_motor_angle / 10,      /* 角度整数部分 */
                   (g_motor_angle % 10 >= 0) ? (g_motor_angle % 10) : (-(g_motor_angle % 10)),  /* 角度小数部分绝对值 */
                   g_motor_speed,           /* 转速(rpm) */
                   g_enc_total,             /* 编码器计数 */
                   g_status_flags);         /* 状态标志 */
        
        }
    }
}

/*=============================================================================
 * UART1 中断（调试串口）
 *===========================================================================*/
void UART1_ISR(void) interrupt 4
{
    unsigned char dat;
    if (RI)
    {
        RI  = 0;
        dat = SBUF;
    }
    if (TI)
    {
        TI = 0;
    }
}

/*=============================================================================
 * UART2 中断（Modbus RTU）
 *===========================================================================*/
void UART2_ISR(void) interrupt 8
{
    if (S2CON & 0x01)                   /* S2RI：收到一个字节 */
    {
        S2CON &= ~0x01;
        Modbus_RxByte(S2BUF);          /* 直接送 Modbus 状态机 */
    }

    if (S2CON & 0x02)                   /* S2TI：发送完一个字节 */
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

/*=============================================================================
 * Timer0 中断（1ms 时基）
 *===========================================================================*/
void Timer0_ISR(void) interrupt 1
{
    int enc_now;
    int enc_delta;

    ms_tick++;

    /* Modbus 3.5字符超时检测 */
    Modbus_TimerTick();

    /* 按键扫描 */
    Key_Scan();

    /* 编码器采样与计算（每10ms执行一次） */
    if (ms_tick % 10 == 0)
    {
        enc_now   = Encoder_Read();
        enc_delta = enc_now - g_enc_last;
        g_enc_last = enc_now;
        g_enc_cnt  = enc_now;
 
        /* 累计计数（32位，不清零） */
        g_enc_total += (long)enc_delta;
 
        /*
         * 速度换算（输出轴 rpm）：
         *   speed = enc_delta * 6000 / ENC_PPR_OUTPUT
         *         = enc_delta * 6000 / 2640
         *         = enc_delta * 25 / 11
         * 用 long 中间值防止 int 溢出（最大转速100rpm时 enc_delta≈4.4）
         */
        g_motor_speed = (int)((long)enc_delta * 6000L / ENC_PPR_OUTPUT);
 
        /*
         * 角度换算（输出轴，0.1° 精度）：
         *   angle = (enc_total % ENC_PPR_OUTPUT) * 3600 / ENC_PPR_OUTPUT
         *         = (enc_total % 2640) * 3600 / 2640
         * 结果范围：0 ~ 3599（对应 0.0° ~ 359.9°）
         * 负数取模在C中结果为负，加 ENC_PPR_OUTPUT 修正为正值
         */
        {
            long rem = g_enc_total % (long)ENC_PPR_OUTPUT;
            if (rem < 0) rem += (long)ENC_PPR_OUTPUT;
            g_motor_angle = (int)(rem * 3600L / ENC_PPR_OUTPUT);
        }
    }

    /* OLED 刷新标志（每20ms） */
    if (ms_tick % 20 == 0)
    {
        disp_flag = 1;
    }

    if (ms_tick >= 10000) ms_tick = 0;
}
