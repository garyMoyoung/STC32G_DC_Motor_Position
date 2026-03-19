/*==============================================================================
 * pid.c  —  位置式PID控制器实现
 *
 * Kp/Ki/Kd 放大100倍整数存储，计算时再除以100，避免浮点运算。
 * 例如：Kp=2.0 → 传入 200
 *       Ki=0.20 → 传入 20
 *       Kd=0.10 → 传入 10
 *==============================================================================*/

#include "pid.h"

/*=============================================================================
 * PID_Init  —  初始化PID参数
 * kp/ki/kd : 实际系数×100的整数值
 * out_max/min : 输出限幅
 * dead_band   : 死区，误差绝对值小于等于此值时输出为0
 *===========================================================================*/
void PID_Init(PID_t *pid,
              int kp, int ki, int kd,
              long out_max, long out_min,
              int dead_band)
{
    pid->Kp        = kp;
    pid->Ki        = ki;
    pid->Kd        = kd;
    pid->out_max   = out_max;
    pid->out_min   = out_min;
    pid->dead_band = dead_band;
    PID_Reset(pid);
}

/*=============================================================================
 * PID_Reset  —  清零历史状态（切换模式/停机时调用）
 *===========================================================================*/
void PID_Reset(PID_t *pid)
{
    pid->e_prev   = 0;
    pid->integral = 0;
    pid->output   = 0;
}

/*=============================================================================
 * PID_Calc  —  位置式PID计算，每个控制周期调用一次
 *
 * 参数：
 *   setpoint : 目标值
 *   feedback : 反馈值（实际值）
 *
 * 返回：
 *   本次输出值（已限幅）
 *
 * 计算过程（Kp/Ki/Kd均已×100存储，最后÷100还原）：
 *   u = (Kp*e + Ki*integral + Kd*(e - e_prev)) / 100
 *
 * 抗积分饱和：
 *   当 Ki>0 时，将 integral 限幅在 ±(out_max*100/Ki)，
 *   使积分项贡献不超过输出限幅，防止积分过大导致超调。
 *===========================================================================*/
long PID_Calc(PID_t *pid, int setpoint, int feedback)
{
    int  err;
    int  derivative;
    long output;

    /* 计算误差 */
    err = setpoint - feedback;

    /* 死区处理：误差在死区内时视为0 */
    if (err <= pid->dead_band && err >= -pid->dead_band)
        err = 0;

    /* 积分累加 */
    pid->integral += (long)err;

    /* 抗积分饱和：限制积分累加器，防止积分项溢出或超调 */
    if (pid->Ki > 0)
    {
        long ilimit = pid->out_max * 100L / (long)pid->Ki;
        if (pid->integral >  ilimit) pid->integral =  ilimit;
        if (pid->integral < -ilimit) pid->integral = -ilimit;
    }

    /* 微分项（基于误差变化率） */
    derivative = err - pid->e_prev;
    pid->e_prev = err;

    /* 位置式PID输出（Kp/Ki/Kd均×100存储，整体÷100还原） */
    output = (  (long)pid->Kp * (long)err
              + (long)pid->Ki * pid->integral
              + (long)pid->Kd * (long)derivative
             ) / 100L;

    /* 输出限幅 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->output = output;
    return output;
}
