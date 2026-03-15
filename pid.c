/*==============================================================================
 * pid.c  —  增量式PID控制器实现
 *
 * Kp/Ki/Kd 放大100倍整数存储，计算时再除以100，避免浮点运算。
 * 例如：Kp=1.5 → 传入 150
 *       Ki=0.05 → 传入 5
 *       Kd=0.02 → 传入 2
 *==============================================================================*/

#include "pid.h"

/*=============================================================================
 * PID_Init  —  初始化PID参数
 * kp/ki/kd : 实际系数×100的整数值
 * out_max/min : 输出限幅
 * dead_band   : 死区，误差绝对值小于此值时输出不变
 *===========================================================================*/
void PID_Init(PID_t *pid,
              int kp, int ki, int kd,
              long out_max, long out_min,
              int dead_band)
{
    pid->Kp       = kp;
    pid->Ki       = ki;
    pid->Kd       = kd;
    pid->out_max  = out_max;
    pid->out_min  = out_min;
    pid->dead_band = dead_band;
    PID_Reset(pid);
}

/*=============================================================================
 * PID_Reset  —  清零历史状态（切换模式/停机时调用）
 *===========================================================================*/
void PID_Reset(PID_t *pid)
{
    pid->e0     = 0;
    pid->e1     = 0;
    pid->e2     = 0;
    pid->output = 0;
}

/*=============================================================================
 * PID_Calc  —  增量式PID计算，每个控制周期调用一次
 *
 * 参数：
 *   setpoint : 目标值
 *   feedback : 反馈值（实际值）
 *
 * 返回：
 *   本次累计输出值（已限幅）
 *
 * 计算过程（Kp/Ki/Kd均已×100存储，最后÷100还原）：
 *   delta_u = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *   u += delta_u / 100
 *===========================================================================*/
long PID_Calc(PID_t *pid, int setpoint, int feedback)
{
    long delta_u;
    int  err;

    /* 计算误差 */
    err = setpoint - feedback;

    /* 死区处理：误差在死区内，保持输出不变 */
    if (err < 0) err = -err > pid->dead_band ? setpoint - feedback : 0;
    else         err =  err > pid->dead_band ? setpoint - feedback : 0;

    /* 移位历史误差 */
    pid->e2 = pid->e1;
    pid->e1 = pid->e0;
    pid->e0 = err;

    /*
     * 增量计算（Kp/Ki/Kd均×100存储，整体再÷100）
     * 用 long 防止中间溢出
     */
    delta_u = (long)pid->Kp * (pid->e0 - pid->e1)
            + (long)pid->Ki *  pid->e0
            + (long)pid->Kd * (pid->e0 - 2 * pid->e1 + pid->e2);
    delta_u /= 100;

    /* 累加到输出 */
    pid->output += delta_u;

    /* 输出限幅 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}
