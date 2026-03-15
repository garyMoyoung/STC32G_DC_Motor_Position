/*==============================================================================
 * modbus.c  —  Modbus RTU 从机实现
 * 硬件：STC32G + UART2（P4.6/RXD2, P4.7/TXD2）+ P1.2/RS485方向控制
 *
 * 支持功能码：
 *   0x01  Read Coils             读线圈
 *   0x03  Read Holding Registers 读保持寄存器
 *   0x05  Write Single Coil      写单个线圈
 *   0x06  Write Single Register  写单个寄存器
 *   0x10  Write Multiple Regs    写多个寄存器
 *============================================================================*/

#include "rs485.h"
#include "uart.h"
#include <STC32G.H>
#include <string.h>

/*=============================================================================
 * 数据存储区
 *===========================================================================*/
unsigned int  modbus_regs[MODBUS_REG_NUM];   // 保持寄存器
unsigned char modbus_coils = 0x00;           // 线圈（每bit一个线圈）

/*=============================================================================
 * 内部状态
 *===========================================================================*/
static unsigned char rx_buf[MODBUS_BUF_SIZE];   // 接收缓冲区
static unsigned char rx_len = 0;                // 已接收字节数
static unsigned char rx_done = 0;               // 帧接收完成标志

static unsigned char tx_buf[MODBUS_BUF_SIZE];   // 发送缓冲区
static unsigned char tx_len = 0;                // 待发送字节数

static unsigned char timeout_cnt = 0;           // 超时计数（单位：1ms tick）
static unsigned char rx_active  = 0;            // 正在接收帧中

/*=============================================================================
 * CRC16（Modbus标准多项式 0xA001）
 *===========================================================================*/
static unsigned int CRC16(unsigned char *buf, unsigned char len)
{
    unsigned int  crc = 0xFFFF;
    unsigned char i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*=============================================================================
 * 发送回复帧（含CRC，发送完毕后切回接收模式）
 *===========================================================================*/
static void SendResponse(unsigned char len)
{
    unsigned int crc;
    unsigned char i;

    crc = CRC16(tx_buf, len);
    tx_buf[len]     = (unsigned char)(crc & 0xFF);
    tx_buf[len + 1] = (unsigned char)(crc >> 8);
    tx_len = len + 2;

    RS485_TX();   // 空操作，自动流控
    for (i = 0; i < tx_len; i++)
        UART2_SendByte(tx_buf[i]);
    RS485_RX();   // 空操作，自动流控
}

/*=============================================================================
 * 发送异常响应
 *===========================================================================*/
static void SendException(unsigned char func, unsigned char ex_code)
{
    tx_buf[0] = MODBUS_SLAVE_ADDR;
    tx_buf[1] = func | 0x80;           // 功能码最高位置1表示异常
    tx_buf[2] = ex_code;
    SendResponse(3);
}

/*=============================================================================
 * 处理完整 Modbus 帧
 *===========================================================================*/
static void ProcessFrame(void)
{
    unsigned int  crc_recv, crc_calc;
    unsigned char func;
    unsigned int  start_addr, quantity;
    unsigned char byte_cnt;
    unsigned int  i;
    unsigned char coil_val;

    /* --- 1. 最短帧长检查（至少4字节：地址+功能码+CRC） --- */
    if (rx_len < 4)
        return;

    /* --- 2. 地址过滤（0xFF 为广播地址，只执行不回复；目前忽略广播） --- */
    if (rx_buf[0] != MODBUS_SLAVE_ADDR)
        return;

    /* --- 3. CRC 校验 --- */
    crc_recv = ((unsigned int)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    crc_calc = CRC16(rx_buf, rx_len - 2);
    if (crc_recv != crc_calc)
        return;                         // CRC 错误，丢弃，不回复

    func       = rx_buf[1];
    start_addr = ((unsigned int)rx_buf[2] << 8) | rx_buf[3];
    quantity   = ((unsigned int)rx_buf[4] << 8) | rx_buf[5];

    /* --- 4. 分功能码处理 --- */
    switch (func)
    {
        /*--------------------------------------------------------------------
         * FC=0x01  Read Coils
         * 请求帧：[地址][01][起始高][起始低][数量高][数量低][CRC-L][CRC-H]
         * 响应帧：[地址][01][字节数][数据…][CRC-L][CRC-H]
         *------------------------------------------------------------------*/
        case FC_READ_COILS:
            if (quantity < 1 || quantity > MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            byte_cnt = (unsigned char)((quantity + 7) / 8);
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_READ_COILS;
            tx_buf[2] = byte_cnt;
            /* 从 modbus_coils 中提取对应位 */
            for (i = 0; i < byte_cnt; i++)
                tx_buf[3 + i] = (modbus_coils >> (start_addr + i * 8)) & 0xFF;
            SendResponse(3 + byte_cnt);
            break;

        /*--------------------------------------------------------------------
         * FC=0x03  Read Holding Registers
         *------------------------------------------------------------------*/
        case FC_READ_HOLDING_REGS:
            if (quantity < 1 || quantity > 125)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            byte_cnt = (unsigned char)(quantity * 2);
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_READ_HOLDING_REGS;
            tx_buf[2] = byte_cnt;
            for (i = 0; i < quantity; i++)
            {
                tx_buf[3 + i * 2]     = (unsigned char)(modbus_regs[start_addr + i] >> 8);
                tx_buf[3 + i * 2 + 1] = (unsigned char)(modbus_regs[start_addr + i] & 0xFF);
            }
            SendResponse(3 + byte_cnt);
            break;

        /*--------------------------------------------------------------------
         * FC=0x05  Write Single Coil
         * 写值：0xFF00=ON, 0x0000=OFF
         *------------------------------------------------------------------*/
        case FC_WRITE_SINGLE_COIL:
            if (start_addr >= MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            coil_val = (rx_buf[4] == 0xFF) ? 1 : 0;
            if (coil_val)
                modbus_coils |=  (1 << start_addr);
            else
                modbus_coils &= ~(1 << start_addr);

            /* 原样回显请求帧（去掉CRC部分，SendResponse会重新加CRC） */
            tx_buf[0] = rx_buf[0];
            tx_buf[1] = rx_buf[1];
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[3];
            tx_buf[4] = rx_buf[4];
            tx_buf[5] = rx_buf[5];
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * FC=0x06  Write Single Register
         *------------------------------------------------------------------*/
        case FC_WRITE_SINGLE_REG:
            if (start_addr >= MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            modbus_regs[start_addr] = quantity;   // quantity 字段即寄存器值
            tx_buf[0] = rx_buf[0];
            tx_buf[1] = rx_buf[1];
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[3];
            tx_buf[4] = rx_buf[4];
            tx_buf[5] = rx_buf[5];
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * FC=0x10  Write Multiple Registers
         * 请求帧：[地址][10][起始高][起始低][数量高][数量低][字节数][数据…][CRC]
         *------------------------------------------------------------------*/
        case FC_WRITE_MULTI_REGS:
            if (quantity < 1 || quantity > 123)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            byte_cnt = rx_buf[6];
            if (byte_cnt != quantity * 2)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            for (i = 0; i < quantity; i++)
            {
                modbus_regs[start_addr + i] =
                    ((unsigned int)rx_buf[7 + i * 2] << 8) | rx_buf[7 + i * 2 + 1];
            }
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_WRITE_MULTI_REGS;
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[3];
            tx_buf[4] = rx_buf[4];
            tx_buf[5] = rx_buf[5];
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * 不支持的功能码
         *------------------------------------------------------------------*/
        default:
            SendException(func, EX_ILLEGAL_FUNCTION);
            break;
    }
}

/*=============================================================================
 * 公共接口实现
 *===========================================================================*/

void Modbus_Init(void)
{
    RS485_RX();             // 默认接收模式
    rx_len      = 0;
    rx_done     = 0;
    rx_active   = 0;
    timeout_cnt = 0;
}

/*-----------------------------------------------------------------------------
 * 每 1ms 在 Timer0_ISR 中调用
 *---------------------------------------------------------------------------*/
void Modbus_TimerTick(void)
{
    if (rx_active)
    {
        timeout_cnt++;
        if (timeout_cnt >= MODBUS_TIMEOUT_MS)
        {
            /* 3.5字符超时，判定帧结束 */
            rx_active   = 0;
            timeout_cnt = 0;
            if (rx_len > 0)
                rx_done = 1;    // 通知 Poll 处理
        }
    }
}

/*-----------------------------------------------------------------------------
 * 每收到1字节在 UART2_ISR 接收分支中调用
 *---------------------------------------------------------------------------*/
void Modbus_RxByte(unsigned char dat)
{
    /* 重置超时计数器（每收到新字节就重新计时） */
    timeout_cnt = 0;
    rx_active   = 1;

    if (rx_len < MODBUS_BUF_SIZE)
        rx_buf[rx_len++] = dat;

    /* 溢出时直接丢弃，等超时后清空 */
    /* 注意：此函数在 UART2_ISR 中调用，禁止在此处使用 Printf 等阻塞输出 */
}

/*-----------------------------------------------------------------------------
 * 主循环中轮询
 *---------------------------------------------------------------------------*/
void Modbus_Poll(void)
{
    if (!rx_done)
        return;

    rx_done = 0;
    ProcessFrame();
    rx_len = 0;             // 清空缓冲，准备下一帧
}
