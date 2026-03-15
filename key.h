#ifndef __KEY_H__
#define __KEY_H__

#include <STC32G.H>

/*=============================================================================
 * 按键 GPIO 定义（P4.4, P4.3, P4.2）
 * STC32G 初始状态：P4M1=0, P4M0=0（准双向/输入模式）
 *===========================================================================*/
sbit KEY1_PIN = P4^4;  // KEY1 - P4.4
sbit KEY2_PIN = P4^3;  // KEY2 - P4.3
sbit KEY3_PIN = P4^2;  // KEY3 - P4.2

/*=============================================================================
 * 按键事件类型定义
 *===========================================================================*/
#define KEY_NONE            0   // 无事件
#define KEY_SINGLE_CLICK    1   // 单击
#define KEY_DOUBLE_CLICK    2   // 双击
#define KEY_LONG_PRESS      3   // 长按
#define KEY_PRESSING        4   // 长按中（持续按下）
#define KEY_RELEASE         5   // 释放

/*=============================================================================
 * 时间参数定义（单位：ms）
 *===========================================================================*/
#define KEY_DEBOUNCE_TIME       20      // 去抖延时（按键稳定判定）
#define KEY_LONG_PRESS_TIME     1000    // 长按判定时间
#define KEY_DOUBLE_CLICK_INTERVAL 300   // 双击识别间隔（两次单击最大时间差）


/*=============================================================================
 * 蜂鸣器支持（已集成到 key.c 中）
 *===========================================================================*/
sbit BUZZER = P1^3;         /* P1.3 有源蜂鸣器 */

/* 蜂鸣器状态 */
#define BUZZER_IDLE             0           /* 空闲 */
#define BUZZER_ON               1           /* 开启中 */
#define BUZZER_OFF              2           /* 关闭中 */

/* 音效类型 */
#define TONE_NONE               0
#define TONE_POWER_ON           1           /* 开机音效 */
#define TONE_SINGLE_CLICK       2           /* 单击提示 */
#define TONE_DOUBLE_CLICK       3           /* 双击提示 */

/*
 * 播放音效（非阻塞，状态机驱动）
 * tone_type: TONE_POWER_ON, TONE_SINGLE_CLICK, TONE_DOUBLE_CLICK等
 * 立即返回，由内部状态机在每1ms自动推进
 */
void Buzzer_PlayTone(unsigned char tone_type);

/*
 * 获取蜂鸣器是否忙碌
 * 返回值：1=忙碌(正在播放), 0=空闲
 */
unsigned char Buzzer_IsBusy(void);


/*=============================================================================
 * 按键结构体
 *===========================================================================*/
typedef struct {
    unsigned char pin_level;        // 当前GPIO读值（0=按下，1=释放）
    unsigned char state;            // 按键状态（0=释放，1=按下）
    unsigned char event;            // 按键事件（上面定义的事件类型）
    
    unsigned int press_time;        // 按键按下的时间戳（ms）
    unsigned int release_time;      // 按键释放的时间戳（ms）
    unsigned int debounce_cnt;      // 去抖计数器
    
    unsigned char click_count;      // 单击计数（用于识别双击）
    unsigned int click_timer;       // 双击识别定时器
    unsigned char debug_flag;   // [新增] 调试标志
} KEY_Struct;


/*=============================================================================
 * 对外接口
 *===========================================================================*/

extern KEY_Struct key1, key2, key3;

/*
 * 初始化按键（在main()中调用）
 */
void Key_Init(void);

/*
 * 按键扫描（在Timer0_ISR中每1ms调用一次）
 */
void Key_Scan(void);

/*
 * 获取按键事件
 * key_id: 1/2/3
 * 返回值：KEY_NONE/KEY_SINGLE_CLICK/KEY_DOUBLE_CLICK/KEY_LONG_PRESS等
 */
unsigned char Key_GetEvent(unsigned char key_id);

/*
 * 清除按键事件
 */
void Key_ClearEvent(unsigned char key_id);

/*
 * 获取按键当前物理状态
 * 返回值：1=释放，0=按下
 */
unsigned char Key_IsPressed(unsigned char key_id);

unsigned char Key_HandleEvent(void);





#endif /* __KEY_H__ */
