#include "key.h"
#include "uart.h"
#include "rs485.h"
#include <STC32G.H>

/*=============================================================================
 * 调节目标（与 main.c 中的 g_key_adj_target 定义一致）
 *   0 = SetSpd   目标转速（modbus_regs[4]，rpm）
 *   1 = SetAng   目标角度（modbus_regs[5]，0.1°）
 * PID 参数（Kp/Ki/Kd）通过上位机调节，按键不再控制
 *===========================================================================*/
#define ADJ_SET_SPD  0
#define ADJ_SET_ANG  1
#define ADJ_NUM      2   /* 循环总数：只有速度和角度 */

/* 各目标调节量限幅 */
#define SPD_MIN  (-999)
#define SPD_MAX    999
#define ANG_MIN  (-3200)    /* 0.1°，对应 ±320° */
#define ANG_MAX   3200


/*
 * 步长表 [目标][0=单击, 1=长按持续, 2=双击]
 * SetSpd: 1 / 5 / 10 rpm
 * SetAng: 10 / 50 / 100 (0.1°，即 1° / 5° / 10°)
 */
static const int adj_steps[2][3] = {
    {  1,   5,  10},   /* SetSpd */
    { 10,  50, 100},   /* SetAng */
};

extern unsigned char  g_key_adj_target;
/*=============================================================================
 * 全局按键对象
 *===========================================================================*/
KEY_Struct key1, key2, key3;
extern volatile unsigned int ms_tick;

typedef struct {
    unsigned char state;            /* BUZZER_IDLE / BUZZER_ON / BUZZER_OFF */
    unsigned char tone_type;        /* 当前播放的音效类型 */
    unsigned int start_ms;          /* 当前段的开始时间（ms_tick） */
    unsigned char seg_index;        /* 当前播放的段序号 */
    unsigned char seg_count;        /* 总段数 */
} Buzzer_t;

static Buzzer_t g_buzzer = {
    BUZZER_IDLE, TONE_NONE, 0, 0, 0
};

/* 音效数据表：每个音效由多个段组成 */
/* 每段：持续时间(ms), 是否开启(1=ON, 0=OFF) */
static const struct {
    unsigned int duration_ms;
    unsigned char is_on;
} g_tone_segments[][8] = {
    /* [TONE_NONE] = {} 空音效 */
    {
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}  // TONE_NONE
    },
    
    /* [TONE_POWER_ON] = 三声上升（短→中→长） */
    {
        {100, 1},   /* 100ms 开 */
        {80, 0},    /* 80ms 关 */
        {150, 1},   /* 150ms 开 */
        {80, 0},    /* 80ms 关 */
        {200, 1},   /* 200ms 开 */
        {0, 0}      /* 结束标记 */
    },
    
    /* [TONE_SINGLE_CLICK] = 单击提示（一短声） */
    {
        {50, 1},    /* 50ms 开 */
        {0, 0}      /* 结束标记 */
    },
    
    /* [TONE_DOUBLE_CLICK] = 双击提示（两短声） */
    {
        {80, 1},    /* 80ms 开 */
        {60, 0},    /* 60ms 关 */
        {80, 1},    /* 80ms 开 */
        {0, 0}      /* 结束标记 */
    }
};

void Buzzer_Update(void)
{
    unsigned int elapsed_ms;
    
    /* 空闲状态，不做任何事 */
    if (g_buzzer.state == BUZZER_IDLE)
        return;
    
    /* 计算当前段已经过的时间 */
    elapsed_ms = ms_tick - g_buzzer.start_ms;
    
    /* 当前段时间已到，切换到下一段 */
    if (elapsed_ms >= g_tone_segments[g_buzzer.tone_type][g_buzzer.seg_index].duration_ms)
    {
        g_buzzer.seg_index++;
        g_buzzer.start_ms = ms_tick;  /* 重置本段开始时间 */
        
        /* 检查是否到达最后一段（结束标记 duration_ms=0） */
        if (g_tone_segments[g_buzzer.tone_type][g_buzzer.seg_index].duration_ms == 0)
        {
            BUZZER = 0;              /* 确保蜂鸣器关闭 */
            g_buzzer.state = BUZZER_IDLE;
            g_buzzer.tone_type = TONE_NONE;
            return;
        }
        
        /* 设置新段的输出状态 */
        if (g_tone_segments[g_buzzer.tone_type][g_buzzer.seg_index].is_on)
            BUZZER = 1;  /* 开启 */
        else
            BUZZER = 0;  /* 关闭 */
    }
}


void Buzzer_PlayTone(unsigned char tone_type)
{
    if (tone_type == TONE_NONE || tone_type > TONE_DOUBLE_CLICK)
        return;
    
    g_buzzer.state = BUZZER_ON;
    g_buzzer.tone_type = tone_type;
    g_buzzer.seg_index = 0;
    g_buzzer.start_ms = ms_tick;
    
    /* 立即设置第一段状态 */
    if (g_tone_segments[tone_type][0].is_on)
        BUZZER = 1;
    else
        BUZZER = 0;
}

/*=============================================================================
 * 查询蜂鸣器忙碌状态
 *===========================================================================*/
unsigned char Buzzer_IsBusy(void)
{
    return (g_buzzer.state != BUZZER_IDLE);
}

void Key_Init(void)
{
    // P4.2, P4.3, P4.4 已在 Sys_init() 中配置为准双向/输入模式
    // P4M1 = 0x00, P4M0 = 0x00（初始全为准双向）
    // 无需额外配置，直接读取GPIO值
    
        /* P1.3 配置为推挽输出（蜂鸣器） */
    P1M1 &= ~0x08;
    P1M0 |=  0x08;

    // 初始化按键1
    key1.pin_level = 1;
    key1.state = 0;          // 初始无按键
    key1.event = KEY_NONE;
    key1.press_time = 0;
    key1.release_time = 0;
    key1.debounce_cnt = 0;
    key1.click_count = 0;
    key1.click_timer = 0;
    
    // 初始化按键2
    key2.pin_level = 1;
    key2.state = 0;
    key2.event = KEY_NONE;
    key2.press_time = 0;
    key2.release_time = 0;
    key2.debounce_cnt = 0;
    key2.click_count = 0;
    key2.click_timer = 0;
    
    // 初始化按键3
    key3.pin_level = 1;
    key3.state = 0;
    key3.event = KEY_NONE;
    key3.press_time = 0;
    key3.release_time = 0;
    key3.debounce_cnt = 0;
    key3.click_count = 0;
    key3.click_timer = 0;
    
    Printf("[KEY] Init OK\r\n");
}

/*=============================================================================
 * 单个按键状态机处理
 * 
 * 状态转移：
 *   0(释放) -> 1(按下) -> 判定为单击?(有),再判双击?(有) -> 0(释放)
 *                      -> 长按判定?(有)长按 -> 0(释放)
 *
 * 双击逻辑：
 *   第一次单击后，在300ms内再按一次即为双击
 *===========================================================================*/
static void Key_ScanSingle(KEY_Struct *pkey, unsigned char pin)
{
    pkey->pin_level = pin;
    
    // ========== 按键按下过程 ==========
    if (pkey->pin_level == 0)  // GPIO为0
    {
        if (pkey->state == 0)  // 从释放状态进入
        {
            pkey->debounce_cnt++;
            if (pkey->debounce_cnt >= KEY_DEBOUNCE_TIME)
            {
                pkey->state = 1;
                pkey->press_time = 0;
                pkey->debounce_cnt = 0;
            }
        }
        else if (pkey->state == 1)
        {
            pkey->press_time++;
            
            // 长按判定（1000ms）
            if (pkey->press_time == KEY_LONG_PRESS_TIME)
            {
                pkey->event = KEY_LONG_PRESS;
                pkey->click_count = 0;  // 清除单击计数，禁止后续双击识别
                pkey->click_timer = 0;  // 停止双击定时器
            }
            // 长按持续中（每100ms）
            else if (pkey->press_time > KEY_LONG_PRESS_TIME && 
                     pkey->press_time % 100 == 0)
            {
                pkey->event = KEY_PRESSING;
            }
        }
    }

    // ========== 按键释放过程 ==========
    else  // GPIO为1（释放）
    {
        if (pkey->state == 1)
        {
            pkey->debounce_cnt++;
            if (pkey->debounce_cnt >= KEY_DEBOUNCE_TIME)
            {
                pkey->state = 0;
                pkey->debounce_cnt = 0;
                
                // 判定是短按还是长按
                if (pkey->press_time < KEY_LONG_PRESS_TIME)
                {
                    // 短按逻辑
                    pkey->click_count++;
                    
                    if (pkey->click_count == 1)
                    {
                        // 第一次单击：立即发送事件，启动双击检测
                        pkey->event = KEY_SINGLE_CLICK;
                        pkey->click_timer = KEY_DOUBLE_CLICK_INTERVAL;
                    }
                    else if (pkey->click_count >= 2)
                    {
                        // 第二次单击：立即发送双击事件，停止定时器
                        pkey->event = KEY_DOUBLE_CLICK;
                        pkey->click_timer = 0;  // 关键：停止定时器，防止后续的单击重复
                        pkey->click_count = 0;
                    }
                }
                else
                {
                    // 长按释放
                    pkey->event = KEY_RELEASE;
                    pkey->click_count = 0;
                    pkey->click_timer = 0;
                }
            }
        }
        else
        {
            pkey->debounce_cnt = 0;
        }
    }

    // ========== 双击定时器倒计时 ==========
    if (pkey->click_timer > 0)
    {
        pkey->click_timer--;
        
        // 定时器超时（300ms内没有第二次点击）
        if (pkey->click_timer == 0)
        {
            // 超时后只需清除单击计数
            // 不再重新设置 event = KEY_SINGLE_CLICK！
            // 因为第一次释放时已经发送过单击事件并被主循环处理了，
            // 如果这里再设一次会导致单击跳两次。
            pkey->click_count = 0;
        }
    }
}

/*=============================================================================
 * 按键扫描（在 Timer0_ISR 中每 1ms 调用一次）
 *===========================================================================*/
void Key_Scan(void)
{
    Key_ScanSingle(&key1, KEY1_PIN);
    Key_ScanSingle(&key2, KEY2_PIN);
    Key_ScanSingle(&key3, KEY3_PIN);
}

/*=============================================================================
 * 获取按键事件
 *===========================================================================*/
unsigned char Key_GetEvent(unsigned char key_id)
{
    switch (key_id)
    {
        case 1:
            return key1.event;
        case 2:
            return key2.event;
        case 3:
            return key3.event;
        default:
            return KEY_NONE;
    }
}

/*=============================================================================
 * 清除按键事件
 *===========================================================================*/
void Key_ClearEvent(unsigned char key_id)
{
    switch (key_id)
    {
        case 1:
            key1.event = KEY_NONE;
            break;
        case 2:
            key2.event = KEY_NONE;
            break;
        case 3:
            key3.event = KEY_NONE;
            break;
    }
}

/*=============================================================================
 * 获取按键当前物理状态（0=按下，1=释放）
 *===========================================================================*/
unsigned char Key_IsPressed(unsigned char key_id)
{
    switch (key_id)
    {
        case 1:
            return (key1.state == 1);  // 1表示当前按下
        case 2:
            return (key2.state == 1);
        case 3:
            return (key3.state == 1);
        default:
            return 0;
    }
}

/*=============================================================================
 * Key_AdjustTarget  —  对当前调节目标施加步长 step（正负均可）
 *===========================================================================*/
static void Key_AdjustTarget(int step)
{
    int sv;
    switch (g_key_adj_target)
    {
        case ADJ_SET_SPD:
            sv = (int)modbus_regs[4] + step;
            if (sv > SPD_MAX) sv = SPD_MAX;
            if (sv < SPD_MIN) sv = SPD_MIN;
            modbus_regs[4] = (unsigned int)sv;
            Printf("SetSpd=%d\r\n", sv);
            break;

        case ADJ_SET_ANG:
            sv = (int)modbus_regs[5] + step;
            if (sv > ANG_MAX) sv = ANG_MAX;
            if (sv < ANG_MIN) sv = ANG_MIN;
            modbus_regs[5] = (unsigned int)sv;
            Printf("SetAng=%d(0.1deg)\r\n", sv);
            break;

        default:
            break;
    }
}

/*=============================================================================
 * 处理所有按键事件（主循环调用）
 *
 * KEY1 单击：循环切换调节目标（SetSpd → SetAng → Kp → Ki → SetSpd...）
 * KEY2 单击：+小步  长按持续：+中步  双击：+大步
 * KEY3 单击：-小步  长按持续：-中步  双击：-大步
 *
 * 步长见 adj_steps[target][0/1/2]
 *===========================================================================*/
unsigned char Key_HandleEvent(void)
{
    unsigned char event;

    /* ── KEY1：切换调节目标 ── */
    event = Key_GetEvent(1);
    if (event == KEY_SINGLE_CLICK)
    {
        g_key_adj_target = (g_key_adj_target + 1) % ADJ_NUM;
        Buzzer_PlayTone(TONE_SINGLE_CLICK);
        Printf("Adj target: %d\r\n", (int)g_key_adj_target);
        Key_ClearEvent(1);
        return 1;
    }
    Key_ClearEvent(1);

    /* ── KEY2：增大 ── */
    event = Key_GetEvent(2);
    if (event == KEY_SINGLE_CLICK)
    {
        Key_AdjustTarget(adj_steps[g_key_adj_target][0]);
        Buzzer_PlayTone(TONE_SINGLE_CLICK);
        Key_ClearEvent(2);
        return 1;
    }
    else if (event == KEY_DOUBLE_CLICK)
    {
        Key_AdjustTarget(adj_steps[g_key_adj_target][2]);
        Buzzer_PlayTone(TONE_DOUBLE_CLICK);
        Key_ClearEvent(2);
        return 1;
    }
    else if (event == KEY_PRESSING)
    {
        Key_AdjustTarget(adj_steps[g_key_adj_target][1]);
        Key_ClearEvent(2);
        return 1;
    }
    Key_ClearEvent(2);

    /* ── KEY3：减小 ── */
    event = Key_GetEvent(3);
    if (event == KEY_SINGLE_CLICK)
    {
        Key_AdjustTarget(-adj_steps[g_key_adj_target][0]);
        Buzzer_PlayTone(TONE_SINGLE_CLICK);
        Key_ClearEvent(3);
        return 1;
    }
    else if (event == KEY_DOUBLE_CLICK)
    {
        Key_AdjustTarget(-adj_steps[g_key_adj_target][2]);
        Buzzer_PlayTone(TONE_DOUBLE_CLICK);
        Key_ClearEvent(3);
        return 1;
    }
    else if (event == KEY_PRESSING)
    {
        Key_AdjustTarget(-adj_steps[g_key_adj_target][1]);
        Key_ClearEvent(3);
        return 1;
    }
    Key_ClearEvent(3);

    return 0;
}
