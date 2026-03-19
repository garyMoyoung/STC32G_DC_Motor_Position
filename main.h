#ifndef __MAIN_H__
#define __MAIN_H__

#define  DELAY_TIME   2000
#define FOSC    33177600UL

#define UART1_BAUD      115200
#define UART1_RELOAD    (65536 - FOSC / 4 / UART1_BAUD)

#define UART2_BAUD      115200

#define UART2_RELOAD    (65536 - FOSC / 4 / UART2_BAUD)    // = 65464 (0xFFB8)


#define PWM_FREQ        10000
/*
 * 边沿对齐模式（CMS=00）计数器从0向上计到ARR，PWM频率=FOSC/ARR
 * 所以 ARR = FOSC / PWM_FREQ = 3317
 *
 * 双半桥驱动占空比含义（互补输出）：
 *   CCR = 0          → PWM1P=0%  PWM1N=100% → 最大正转
 *   CCR = PWM_MID    → PWM1P=50% PWM1N=50%  → 停车（零速中点）
 *   CCR = PWM_ARR    → PWM1P=100% PWM1N=0%  → 最大反转
 */
#define PWM_ARR         (FOSC / PWM_FREQ)                   // = 3317
#define PWM_MID         (PWM_ARR / 2)                       // = 1658  停车中点
#define PWM_DEAD_TIME   60      // 死区时间约 60/33.1776M ≈ 1.8us

#define T0_RELOAD   (65536 - FOSC / 1000)      // = 32359



#endif /* __MAIN_H__ */