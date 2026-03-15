/*==============================================================================
 * key.c  —  按键检测模块（支持单击、双击、长按）
 * 硬件：STC32G + P4.4(KEY1)/P4.3(KEY2)/P4.2(KEY3)
 * 功能：去抖、单击、双击、长按检测
 *============================================================================*/

#include "key.h"
#include "uart.h"

/*=============================================================================
 * 全局按键对象
 *===========================================================================*/
KEY_Struct key1, key2, key3;

/*=============================================================================
 * 按键初始化
 *===========================================================================*/
void Key_Init(void)
{
    // P4.2, P4.3, P4.4 已在 Sys_init() 中配置为准双向/输入模式
    // P4M1 = 0x00, P4M0 = 0x00（初始全为准双向）
    // 无需额外配置，直接读取GPIO值
    
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
            // 只有单击计数为1时才确认为单击
            if (pkey->click_count == 1)
            {
                // 重要改动：检查event是否已经被clear过了
                // 只在event为KEY_NONE时重新设置，否则说明已被处理
                if (pkey->event != KEY_SINGLE_CLICK)
                {
                    pkey->event = KEY_SINGLE_CLICK;
                }
                pkey->click_count = 0;
            }
            else if (pkey->click_count >= 2)
            {
                // 双击已处理，清除计数
                pkey->click_count = 0;
            }
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

static unsigned char last_event1 = KEY_NONE;
static unsigned char last_event2 = KEY_NONE;
static unsigned char last_event3 = KEY_NONE;

/*=============================================================================
 * 处理所有按键事件（主循环调用）
 * 包含KEY1/KEY2/KEY3的所有事件处理逻辑
 *=============================================================================*/
unsigned char Key_HandleEvent(void)
{
    unsigned char event;
    
    /* ===== KEY1 事件处理 ===== */
    event = Key_GetEvent(1);
    
    // 过滤逻辑：如果这帧是双击，则忽略上一帧的单击
    if (event == KEY_DOUBLE_CLICK && last_event1 == KEY_SINGLE_CLICK)
    {
        // 这是双击，不处理单击事件，直接处理双击
        Printf("[KEY1] Double Click\r\n");
        // TODO: 添加双击功能代码
        Key_ClearEvent(1);
        last_event1 = KEY_DOUBLE_CLICK;
        return 1;
    }
    
    // 正常流程
    if (event == KEY_SINGLE_CLICK)
    {
        Printf("[KEY1] Single Click\r\n");
        // TODO: 添加单击功能代码
        Key_ClearEvent(1);
        last_event1 = KEY_SINGLE_CLICK;
        return 1;
    }
    else if (event == KEY_DOUBLE_CLICK)
    {
        Printf("[KEY1] Double Click\r\n");
        // TODO: 添加双击功能代码
        Key_ClearEvent(1);
        last_event1 = KEY_DOUBLE_CLICK;
        return 1;
    }
    else if (event == KEY_LONG_PRESS)
    {
        Printf("[KEY1] Long Press\r\n");
        Key_ClearEvent(1);
        last_event1 = KEY_LONG_PRESS;
        return 1;
    }
    else if (event == KEY_RELEASE)
    {
        Printf("[KEY1] Released\r\n");
        Key_ClearEvent(1);
        last_event1 = KEY_RELEASE;
        return 1;
    }
    
    last_event1 = KEY_NONE;
    
    /* ===== KEY2 事件处理 ===== */
    event = Key_GetEvent(2);
    
    if (event == KEY_DOUBLE_CLICK && last_event2 == KEY_SINGLE_CLICK)
    {
        Printf("[KEY2] Double Click\r\n");
        Key_ClearEvent(2);
        last_event2 = KEY_DOUBLE_CLICK;
        return 1;
    }
    
    if (event == KEY_SINGLE_CLICK)
    {
        Printf("[KEY2] Single Click\r\n");
        Key_ClearEvent(2);
        last_event2 = KEY_SINGLE_CLICK;
        return 1;
    }
    else if (event == KEY_DOUBLE_CLICK)
    {
        Printf("[KEY2] Double Click\r\n");
        Key_ClearEvent(2);
        last_event2 = KEY_DOUBLE_CLICK;
        return 1;
    }
    else if (event == KEY_LONG_PRESS)
    {
        Printf("[KEY2] Long Press\r\n");
        Key_ClearEvent(2);
        last_event2 = KEY_LONG_PRESS;
        return 1;
    }
    
    last_event2 = KEY_NONE;
    
    /* ===== KEY3 事件处理 ===== */
    event = Key_GetEvent(3);
    
    if (event == KEY_DOUBLE_CLICK && last_event3 == KEY_SINGLE_CLICK)
    {
        Printf("[KEY3] Double Click\r\n");
        Key_ClearEvent(3);
        last_event3 = KEY_DOUBLE_CLICK;
        return 1;
    }
    
    if (event == KEY_SINGLE_CLICK)
    {
        Printf("[KEY3] Single Click\r\n");
        Key_ClearEvent(3);
        last_event3 = KEY_SINGLE_CLICK;
        return 1;
    }
    else if (event == KEY_DOUBLE_CLICK)
    {
        Printf("[KEY3] Double Click\r\n");
        Key_ClearEvent(3);
        last_event3 = KEY_DOUBLE_CLICK;
        return 1;
    }
    else if (event == KEY_LONG_PRESS)
    {
        Printf("[KEY3] Long Press\r\n");
        Key_ClearEvent(3);
        last_event3 = KEY_LONG_PRESS;
        return 1;
    }
    
    last_event3 = KEY_NONE;
    return 0;
}
