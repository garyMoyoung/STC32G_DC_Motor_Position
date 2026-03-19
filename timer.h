#ifndef __TIMER_H__
#define __TIMER_H__

void Timer0_Init(void);
void PWM_Init(void);
void PWM_SetDuty(unsigned int duty);
void PWM_OutputEnable(void);
void PWM_OutputDisable(void);
void PWMB_Encoder_Init(void);
int Encoder_Read(void);



#endif /* __TIMER_H__ */
