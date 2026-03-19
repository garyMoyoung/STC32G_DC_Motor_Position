/*==============================================================================
 * main.c  —  集成 Modbus RTU 协议寄存器映射
 *
 * 寄存器映射（对应通信协议 v1.1）：
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
 *   0x000A  Speed_Kp          速度环Kp（可读写，×100整数）
 *   0x000B  Speed_Ki          速度环Ki（可读写，×100整数）
 *   0x000C  Speed_Kd          速度环Kd（可读写，×100整数）
 *   0x000D  Angle_Kp          角度环Kp（可读写，×100整数）
 *   0x000E  Angle_Ki          角度环Ki（可读写，×100整数）
 *   0x000F  Angle_Kd          角度环Kd（可读写，×100整数）
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
#include "pid.h"

/*=============================================================================
 * 编码器参数（根据硬件确定，修改此处即可）
 *
 *   ENC_LINES      编码器物理线数        = 11
 *   ENC_MULT       AB两相四倍频          = 4
 *   GEAR_RATIO     减速比（输出轴：电机） = 60
 *   ENC_PPR_MOTOR  电机轴每转脉冲数      = 11 × 4 = 44
 *   ENC_PPR_OUTPUT 输出轴每转脉冲数      = 44 × 60 = 2640
 *
 * ── 速度单位选择 ──────────────────────────────────────────────────────────
 *   定义 SPEED_MOTOR_SHAFT → g_motor_speed 为电机轴转速（减速箱前，转速高）
 *   注释掉                 → g_motor_speed 为输出轴转速（减速箱后，转速低）
 *
 *   注意：切换单位后 PID 速度环参数（Kp/Ki）需要重新调整！
 *         电机轴转速 ≈ 输出轴转速 × GEAR_RATIO（本例 ×60）
 *
 * ── 参数校准方法 ──────────────────────────────────────────────────────────
 *   1. 上电后转动输出轴整整 N 圈（如 N=1）
 *   2. 通过 Modbus 读 Encoder_Count（0x0002/0x0003）得到累计脉冲数 P
 *   3. ENC_PPR_OUTPUT_实际 = P / N
 *      若 P ≈ 2640 → 参数正确
 *      若 P ≈ 44   → 编码器在输出轴上，应将 GEAR_RATIO 改为 1
 *      若比例不对   → 修正 ENC_LINES 或 GEAR_RATIO
 *   4. 串口调试输出中会实时打印 enc_delta，可用于验证
 *
 * ── 速度公式（采样周期 10ms）─────────────────────────────────────────────
 *   输出轴: speed(rpm) = enc_delta * 6000 / ENC_PPR_OUTPUT
 *   电机轴: speed(rpm) = enc_delta * 6000 / ENC_PPR_MOTOR
 *
 * 角度公式（输出轴，0.1° 精度）：
 *   angle(0.1°) = (enc_total % ENC_PPR_OUTPUT) * 3600 / ENC_PPR_OUTPUT
 *===========================================================================*/
#define ENC_LINES           11
#define ENC_MULT            4
#define GEAR_RATIO          60
#define ENC_PPR_MOTOR       (ENC_LINES * ENC_MULT)          /* 44  脉冲/电机转 */
#define ENC_PPR_OUTPUT      (ENC_PPR_MOTOR * GEAR_RATIO)    /* 2640 脉冲/输出轴转 */

/*
 * 注释掉下面这行 → 输出轴转速（减速后）
 * 保留下面这行   → 电机轴转速（减速前，即电机实际转速）
 */
/* #define SPEED_MOTOR_SHAFT */   /* 注释 = 输出轴转速；取消注释 = 电机轴转速 */

/*
 * 编码器方向（决定 g_motor_speed 的正负与电机物理方向的对应关系）
 *   ENC_DIR =  1：enc_delta 正 = 物理正转（默认）
 *   ENC_DIR = -1：enc_delta 负 = 物理正转（编码器 A/B 相序与电机方向相反时使用）
 *
 * 判断方法：手动给一个正转 PWM（模式2写 Duty < PWM_MID），若串口 dEnc 为负值则改为 -1
 */
#define ENC_DIR             (1)    /* 当前编码器相序与电机正转方向相反 */

/* 速度滑动平均窗口（样本数），增大可减少抖动，减小可提升响应 */
#define SPEED_AVG_N         4

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
unsigned char g_pid_adj_target = 0; /* 0=调Kp  1=调Ki */
volatile bit  debug_flag = 0;       /* 50ms 调试打印标志 */
volatile unsigned int ms_tick   = 0;
volatile unsigned int test_pwmb = 0;
unsigned int  g_pwm_duty = 0;       /* 当前 PWM 占空比（Motor_ControlUpdate 更新） */

static int g_enc_last  = 0;         /* 速度计算：上次采样值 */
static long g_speed_acc  = 0;       /* 速度累加器（滑动平均用） */
static unsigned char g_speed_idx = 0; /* 滑动平均计数 */
int           g_enc_delta  = 0;     /* 最近一次10ms采样的脉冲数（调试用） */

sbit TOG = P0^4;

/*=============================================================================
 * PID 控制器实例
 *
 * 速度环（Speed PID）：
 *   控制周期 = 10ms（编码器采样周期）
 *   输入  = g_set_speed（rpm）vs g_motor_speed（rpm）
 *   输出  = PWM 占空比（0 ~ PWM_ARR = 3317）
 *   初始参数（需根据实际电机调试）：
 *     Kp=200（×100=2.00），Ki=5（×100=0.05），Kd=0（暂不加微分）
 *
 * 位置环（Angle PID）：
 *   控制周期 = 10ms
 *   输入  = g_set_angle（0.1°）vs g_motor_angle（0.1°）
 *   输出  = 目标速度限幅至 ±MAX_SPEED_FOR_ANGLE（rpm）
 *   初始参数：Kp=80（×100=0.80），Ki=2（×100=0.02），Kd=10（×100=0.10）
 *
 * ── 调参说明 ──────────────────────────────────────────────────────────────
 * 1. 先单独调速度环（角度模式关闭，手动给定转速）：
 *    逐步增大 Kp 直到响应变快但不震荡，再加小 Ki 消除稳态误差
 * 2. 速度环稳定后再调位置环：
 *    逐步增大位置环 Kp，加 Kd 抑制超调
 * 3. 参数调整：修改 PID_SPEED_KP/KI/KD 和 PID_ANGLE_KP/KI/KD 宏即可
 *===========================================================================*/

/* 速度环参数（实际值×100）
 * 调参步骤：
 *   1. Ki=0，逐步加大 Kp 直到速度响应快但不振荡
 *   2. Kp 固定后逐步加 Ki 消除稳态误差
 *   3. 若超调明显可适当加 Kd
 * 观察串口输出：Set=目标 Spd=实际 Duty=PWM  dEnc=每10ms脉冲数
 */
/*
 * PWM_MID = 1658（10kHz 边沿对齐），PID 输出限幅 ±1658
 * 调参起点：Ki=0 先调 Kp，让电机能平稳跟上目标转速后再加 Ki 消稳态误差
 *   Kp=200：set=50rpm 首拍 delta_u=100，偏离中点约6%，可驱动电机启动
 *   Ki=20 ：每拍稳态积分，消除稳态误差
 */
#define PID_SPEED_KP    4000    /* Kp = 2.00 */
#define PID_SPEED_KI    100     /* Ki = 0.20，若振荡先清0 */
#define PID_SPEED_KD    0      /* Kd = 0.00 */

/* 位置环参数（实际值×100） */
#define PID_ANGLE_KP    4000      /* Kp = 0.80 */
#define PID_ANGLE_KI    0       /* Ki = 0.02 */
#define PID_ANGLE_KD    0      /* Kd = 0.10 */

/* 位置环输出限幅（最大目标速度，rpm） */
#define MAX_SPEED_FOR_ANGLE   100

static PID_t g_pid_speed;       /* 速度环 PID 实例 */
static PID_t g_pid_angle;       /* 位置环 PID 实例 */

/*=============================================================================
 * Modbus_SyncRegs
 * 主循环每轮调用一次：
 *   - 把最新运行状态刷新到只读寄存器
 *   - 把上位机写入的可写寄存器取回到控制变量
 *   - 同步 PID 参数（上位机写入时更新到 PID 结构体）
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

    // /* 预留寄存器始终为0 */
    // for (k = 10; k < 16; k++)
    //     modbus_regs[k] = 0x0000;

    /* ── 可写寄存器：取回上位机设定值 ── */
    g_set_speed = (int)modbus_regs[4];       /* Set_Speed    */
    g_set_angle = (int)modbus_regs[5];       /* Set_Angle    */
    g_ctrl_mode = (unsigned char)modbus_regs[6]; /* Control_Mode */

    /* ── PID 参数同步（0x000A~0x000F）──
     * 上位机写入后，取回到 PID 结构体
     * 每次同步都更新（开销极小，且保证实时性）
     */
    g_pid_speed.Kp = (int)modbus_regs[0x0A];   /* Speed_Kp */
    g_pid_speed.Ki = (int)modbus_regs[0x0B];   /* Speed_Ki */
    g_pid_speed.Kd = (int)modbus_regs[0x0C];   /* Speed_Kd */
    g_pid_angle.Kp = (int)modbus_regs[0x0D];   /* Angle_Kp */
    g_pid_angle.Ki = (int)modbus_regs[0x0E];   /* Angle_Ki */
    g_pid_angle.Kd = (int)modbus_regs[0x0F];   /* Angle_Kd */

    /* 线圈 Alarm_Clear（bit3）：写1后自动归0，同时清故障标志 */
    if (modbus_coils & (1 << 3))
    {
        g_status_flags = 0;
        modbus_coils  &= ~(1 << 3);
    }
}

/*=============================================================================
 * Motor_ControlUpdate
 * 每10ms在Timer0_ISR编码器采样后调用一次（控制周期与采样周期一致）
 *
 * 模式0 速度模式：速度单环PID
 *   g_set_speed → 速度PID → PWM占空比
 *
 * 模式1 角度模式：位置环+速度环串级PID
 *   g_set_angle → 位置PID → 目标速度 → 速度PID → PWM占空比
 *
 * 模式2 手动模式：直接写PWM_Duty_Raw寄存器
 *===========================================================================*/
/*=============================================================================
 * Motor_ControlUpdate
 *
 * 驱动拓扑：两个半桥（H桥），CCR 决定方向和速度
 *   CCR = PWM_MID (1658) → 50% → 停车（两端电压相等，净电压=0）
 *   CCR < PWM_MID       → 正转（CCR 越小速度越快）
 *   CCR > PWM_MID       → 反转（CCR 越大速度越快）
 *
 * PID 输出约定（范围 ±PWM_MID）：
 *   pid_out > 0 → 需要正转  → CCR = PWM_MID - pid_out  < PWM_MID ✓
 *   pid_out = 0 → 停车      → CCR = PWM_MID                       ✓
 *   pid_out < 0 → 需要反转  → CCR = PWM_MID - pid_out  > PWM_MID ✓
 *
 * g_motor_speed 符号由编码器方向决定：正转 > 0，反转 < 0
 * g_set_speed   符号表示方向：正值正转，负值反转
 *===========================================================================*/
static void Motor_ControlUpdate(void)
{
    long         pid_out;
    int          speed_sp;
    long         duty_l;
    unsigned int pwm_duty = PWM_MID;    /* 默认停车中点 */

    /* Motor_Enable 线圈（bit0）：未使能时关闭PWM输出并复位PID */
    if (!(modbus_coils & (1 << 0)))
    {
        PWM_SetDuty(PWM_MID);       /* 先将占空比置中点，防止重新使能时冲击 */
        PWM_OutputDisable();        /* 关闭PWM输出引脚（ENO=0x00）           */
        g_pwm_duty = PWM_MID;
        PID_Reset(&g_pid_speed);
        PID_Reset(&g_pid_angle);
        return;
    }

    /* Motor_Enable=1：确保PWM输出引脚已打开 */
    PWM_OutputEnable();

    switch (g_ctrl_mode)
    {
        /* ── 模式0：速度单环 ── */
        case 0:
            if (g_set_speed == 0)
            {
                /* 目标为0：停车，复位积分防止下次起步时冲击 */
                PID_Reset(&g_pid_speed);
                /* pwm_duty 已默认为 PWM_MID，直接 break */
                break;
            }

            /*
             * 带符号速度PID：
             *   setpoint = g_set_speed（正转 > 0 / 反转 < 0）
             *   feedback = g_motor_speed（编码器，正转 > 0 / 反转 < 0）
             *   输出范围 ±PWM_MID
             *
             * CCR = PWM_MID - pid_out
             *   pid_out 正 → CCR 小于中点 → 正转 ✓
             *   pid_out 负 → CCR 大于中点 → 反转 ✓
             */
            pid_out = PID_Calc(&g_pid_speed, g_set_speed, g_motor_speed);
            duty_l  = (long)PWM_MID - pid_out;
            if (duty_l < 0L)        duty_l = 0L;
            if (duty_l > PWM_ARR)   duty_l = PWM_ARR;
            pwm_duty = (unsigned int)duty_l;
            break;

        /* ── 模式1：位置串级（位置环 → 速度环） ── */
        case 1:
            /* 第一级：位置环PID
             *   输入：目标角度(0.1°) vs 当前角度(0.1°)
             *   输出：目标速度（rpm），带符号，限幅 ±MAX_SPEED_FOR_ANGLE
             */
            speed_sp = (int)PID_Calc(&g_pid_angle, g_set_angle, g_motor_angle);

            /* 到位判断：误差在±5（0.5°）内认为到位 */
            {
                int angle_err = g_set_angle - g_motor_angle;
                if (angle_err < 0) angle_err = -angle_err;
                if (angle_err <= 5)
                {
                    g_status_flags |= (1 << 0);     /* bit0 到位 */
                    PID_Reset(&g_pid_speed);
                    /* pwm_duty 已默认为 PWM_MID（停车），位置环积分保留 */
                    break;
                }
                else
                {
                    g_status_flags &= ~(1 << 0);
                }
            }

            /* 第二级：速度环PID
             *   setpoint = speed_sp（位置环输出，带符号）
             *   feedback = g_motor_speed（带符号）
             */
            pid_out  = PID_Calc(&g_pid_speed, speed_sp, g_motor_speed);
            duty_l   = (long)PWM_MID - pid_out;
            if (duty_l < 0L)        duty_l = 0L;
            if (duty_l > PWM_ARR)   duty_l = PWM_ARR;
            pwm_duty = (unsigned int)duty_l;
            break;

        /* ── 模式2：手动，直接写 PWM_Duty_Raw（0x0008）── */
        case 2:
            PID_Reset(&g_pid_speed);
            PID_Reset(&g_pid_angle);
            pwm_duty = modbus_regs[8];
            if (pwm_duty > PWM_ARR) pwm_duty = PWM_ARR;
            break;

        default:
            PID_Reset(&g_pid_speed);
            PID_Reset(&g_pid_angle);
            break;
    }

    PWM_SetDuty(pwm_duty);
    g_pwm_duty = pwm_duty;

    /* 方向状态：从编码器速度符号推断（不再依赖 Dir 线圈）
     *   bit2 = 1：正转，bit2 = 0：反转或停止
     */
    if (g_motor_speed > 0)
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

    Key_Init();
    Buzzer_PlayTone(TONE_POWER_ON);
    /* PID 控制器初始化 */
    /*
     * 速度环输出范围 ±PWM_MID（对应 CCR 从 0 到 PWM_ARR）
     *   pid_out > 0 → duty = PWM_MID - pid_out < PWM_MID → 正转
     *   pid_out < 0 → duty = PWM_MID - pid_out > PWM_MID → 反转
     */
    PID_Init(&g_pid_speed,
             PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
             (long)PWM_MID,    /* 输出上限 */
             -(long)PWM_MID,   /* 输出下限 */
             0);               /* 死区 = 2rpm */

    PID_Init(&g_pid_angle,
             PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD,
             MAX_SPEED_FOR_ANGLE,   /* 输出上限（rpm） */
             -MAX_SPEED_FOR_ANGLE,  /* 输出下限（rpm） */
             0);        /* 死区 = 5（对应0.5°） */

    /* 可写寄存器初始值 */
    modbus_regs[4] = 0;     /* Set_Speed  = 0 rpm  */
    modbus_regs[5] = 0;     /* Set_Angle  = 0°     */
    modbus_regs[6] = 0;     /* Control_Mode = 速度 */
    modbus_regs[8] = 0;     /* PWM_Duty_Raw = 0    */

    /* PID参数寄存器初始值（与PID_Init一致，上位机可读取并显示） */
    modbus_regs[0x0A] = PID_SPEED_KP;   /* Speed_Kp */
    modbus_regs[0x0B] = PID_SPEED_KI;   /* Speed_Ki */
    modbus_regs[0x0C] = PID_SPEED_KD;   /* Speed_Kd */
    modbus_regs[0x0D] = PID_ANGLE_KP;   /* Angle_Kp */
    modbus_regs[0x0E] = PID_ANGLE_KI;   /* Angle_Ki */
    modbus_regs[0x0F] = PID_ANGLE_KD;   /* Angle_Kd */

    /* Motor_Enable 线圈默认关闭，等上位机显式使能 */
    modbus_coils = 0x00;

    /* ── 上电初始化信息打印 ── */
    Printf("===== Motor Init =====\n");
    Printf("Enc: lines=%d x%d gear=%d PPR_out=%d\n",
           ENC_LINES, ENC_MULT, GEAR_RATIO, ENC_PPR_OUTPUT);
#ifdef SPEED_MOTOR_SHAFT
    Printf("Speed unit: motor shaft rpm\n");
#else
    Printf("Speed unit: output shaft rpm\n");
#endif
    Printf("AVG_N=%d  PWM_ARR=%d\n", SPEED_AVG_N, PWM_ARR);
    Printf("PID spd: Kp=%d Ki=%d Kd=%d\n",
           PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD);
    Printf("PID ang: Kp=%d Ki=%d Kd=%d\n",
           PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD);
    Printf("Debug: T(ms) Set Spd dEnc Duty\n");
    Printf("======================\n");

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

        /* 3. 按键处理 */
        Key_HandleEvent();

        /* 5. OLED 刷新（每20ms） */
        if (disp_flag)
        {
            disp_flag = 0;
            OLED_BuffShowNum(32, 0, (long)g_motor_speed, 0);
            OLED_BuffShowNum(32, 2, (long)(g_motor_angle / 10), 0);
            OLED_BuffShow();
        }
 
        /* 6. 串口调试打印（每50ms）
         * 字段：
         *   T    = 时间戳 ms
         *   Set  = 目标转速 rpm
         *   Spd  = 实际转速 rpm（滑动平均后）
         *   dEnc = 最近10ms采样的原始脉冲数（观察编码器是否正常计数）
         *   Duty = 当前PWM占空比（0~PWM_ARR=%d）
         *   Ang  = 输出轴角度 0.1°
         *
         * 调参参考：
         *   起步时 dEnc 应在1个脉冲内跟上，Spd 应在2~3个打印周期内收敛到 Set
         *   若 Spd 震荡 → 减小 Kp；若 Spd 响应慢 → 增大 Kp
         *   若稳态误差大 → 增大 Ki
         */
        if (debug_flag)
        {
            debug_flag = 0;
            Printf("Set:Spd:dEnc:Duty:Ang:enc: %d, %d, %d, %u, %d, %d\n",
                   g_set_speed,
                   g_motor_speed,
                   g_enc_delta,
                   g_pwm_duty,
                   g_motor_angle / 10,
                   g_enc_total
                );
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

    /* 蜂鸣器状态更新 */
    Buzzer_Update();
    /* 编码器采样与计算（每10ms执行一次） */
    if (ms_tick % 10 == 0)
    {
        enc_now   = Encoder_Read();
        /* 强制 16 位有符号算术，防止编译器用 32 位 int 导致越界错误
         * ENC_DIR：修正编码器相序与电机方向的关系（±1），见文件顶部宏定义 */
        enc_delta  = (int)(signed short)(enc_now - g_enc_last) * ENC_DIR;
        g_enc_last = enc_now;
        g_enc_cnt  = enc_now;
        g_enc_delta = enc_delta;    /* 保存供调试输出使用（已含方向修正） */

        /* 累计计数（32位，不清零） */
        g_enc_total += (long)enc_delta;

        /*
         * 速度换算：
         *   SPEED_MOTOR_SHAFT 已定义 → 电机轴 rpm（减速箱前）
         *                    未定义  → 输出轴 rpm（减速箱后）
         *
         *   电机轴: speed = enc_delta * 6000 / ENC_PPR_MOTOR  (= /44)
         *   输出轴: speed = enc_delta * 6000 / ENC_PPR_OUTPUT (= /2640)
         *
         *   滑动平均（SPEED_AVG_N 个采样点）减少低速跳变
         */
        g_speed_acc += (long)enc_delta;
        g_speed_idx++;
        if (g_speed_idx >= SPEED_AVG_N)
        {
#ifdef SPEED_MOTOR_SHAFT
            g_motor_speed = (int)(g_speed_acc * 6000L / ENC_PPR_MOTOR / SPEED_AVG_N);
#else
            g_motor_speed = (int)(g_speed_acc * 6000L / ENC_PPR_OUTPUT / SPEED_AVG_N);
#endif
            g_speed_acc = 0;
            g_speed_idx = 0;
        }

        /*
         * 角度换算（输出轴，累计角度，0.1° 精度）：
         *   angle(0.1°) = enc_total * 3600 / ENC_PPR_OUTPUT
         *
         * 上电时 enc_total=0，因此上电位置即为 0°（参考零点）。
         * 结果为有符号累计值，正转为正、反转为负，范围约 ±3276°（±9圈）。
         * 超出 int 范围后溢出，若需更大行程可改为 long 并同步修改 PID 接口。
         */
        {
            long rem = g_enc_total % (long)ENC_PPR_OUTPUT;
            if (rem < 0) rem += (long)ENC_PPR_OUTPUT;
            g_motor_angle = (int)(rem * 3600L / ENC_PPR_OUTPUT);
        }

        /* 编码器采样完毕，立即执行PID控制输出（控制周期=10ms） */
        Motor_ControlUpdate();
    }

    /* OLED 刷新标志（每20ms） */
    if (ms_tick % 20 == 0)
    {
        disp_flag = 1;
    }

    /* 串口调试打印标志（每50ms） */
    if (ms_tick % 50 == 0)
    {
        debug_flag = 1;
    }

    if (ms_tick >= 10000) ms_tick = 0;
}
