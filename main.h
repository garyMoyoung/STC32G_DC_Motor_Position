#ifndef __MAIN_H__
#define __MAIN_H__

#define  DELAY_TIME   2000
#define FOSC    33177600UL

#define UART1_BAUD      115200
#define UART1_RELOAD    (65536 - FOSC / 4 / UART1_BAUD)

#define UART2_BAUD      9600

#define UART2_RELOAD    (65536 - FOSC / 4 / UART2_BAUD)    // = 646720%


#define PWM_FREQ        20000
#define PWM_ARR         (FOSC / PWM_FREQ)                   // = 1658
#define PWM_DEAD_TIME   60      // 死区时间约 60/33.1776M ≈ 1.8us

#define T0_RELOAD   (65536 - FOSC / 1000)      // = 32359



#endif /* __MAIN_H__ */
