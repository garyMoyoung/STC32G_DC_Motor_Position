#ifndef __MAIN_H__
#define __MAIN_H__

#define  DELAY_TIME   2000
#define FOSC    33177600UL

#define UART1_BAUD      115200
#define UART1_RELOAD    (65536 - FOSC / 4 / UART1_BAUD)

#define UART2_BAUD      115200

#define UART2_RELOAD    (65536 - FOSC / 4 / UART2_BAUD)    // = 65464 (0xFFB8)


#define PWM_FREQ        20000
/*
 * 中央对齐模式（CMS=11）计数器从0计到ARR再回到0，等效PWM频率=FOSC/(2*ARR)
 * 所以 ARR = FOSC / (2 * PWM_FREQ) = 829
 * 注意：timer.c 内有 #undef/#define PWM_ARR 829 是旧写法，现已统一到此处
 *
 * 双半桥驱动占空比含义：
 *   CCR = 0          → 0%   → 最大正转
 *   CCR = PWM_MID    → 50%  → 停车（零速中点）
 *   CCR = PWM_ARR    → 100% → 最大反转
 */
#define PWM_ARR         (FOSC / 2 / PWM_FREQ)              // = 829
#define PWM_MID         (PWM_ARR / 2)                       // = 414  停车中点
#define PWM_DEAD_TIME   60      // 死区时间约 60/33.1776M ≈ 1.8us

#define T0_RELOAD   (65536 - FOSC / 1000)      // = 32359



#endif /* __MAIN_H__ */