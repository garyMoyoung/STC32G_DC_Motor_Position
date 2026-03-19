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
/*=============================================================================
 * ProcessFrame —— 处理完整 Modbus RTU 帧
 *
 * 此函数在 Modbus_Poll() 中被调用，当接收到完整帧时执行。
 * 主要步骤：
 *   1. 帧长度和地址校验
 *   2. CRC 校验
 *   3. 解析功能码和参数
 *   4. 根据功能码执行相应操作
 *   5. 发送响应或异常
 *
 * 参数：无（使用全局 rx_buf 和 rx_len）
 * 返回：无
 *===========================================================================*/
static void ProcessFrame(void)
{
    unsigned int  crc_recv, crc_calc;    // 接收到的 CRC 和计算出的 CRC
    unsigned char func;                  // 功能码
    unsigned int  start_addr, quantity;  // 起始地址和数量/值
    unsigned char byte_cnt;              // 数据字节数
    unsigned int  i;                     // 循环变量
    unsigned char coil_val;              // 线圈值（0 或 1）

    /* --- 1. 最短帧长检查（至少4字节：地址+功能码+CRC低+CRC高） --- */
    if (rx_len < 4)
        return;  // 帧太短，丢弃

    /* --- 2. 地址过滤（0xFF 为广播地址，只执行不回复；目前忽略广播） --- */
    if (rx_buf[0] != MODBUS_SLAVE_ADDR)
        return;  // 不是本机地址，忽略

    /* --- 3. CRC 校验 --- */
    crc_recv = ((unsigned int)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];  // 提取接收到的 CRC
    crc_calc = CRC16(rx_buf, rx_len - 2);  // 计算帧数据的 CRC（不含 CRC 字段）
    if (crc_recv != crc_calc)
        return;  // CRC 错误，丢弃，不回复

    /* --- 4. 解析公共字段 --- */
    func       = rx_buf[1];  // 功能码
    start_addr = ((unsigned int)rx_buf[2] << 8) | rx_buf[3];  // 起始地址（高字节在前）
    quantity   = ((unsigned int)rx_buf[4] << 8) | rx_buf[5];  // 数量或值（高字节在前）

    /* --- 5. 分功能码处理 --- */
    switch (func)
    {
        /*--------------------------------------------------------------------
         * FC=0x01  Read Coils（读线圈）
         * 请求帧：[地址][01][起始高][起始低][数量高][数量低][CRC-L][CRC-H]
         * 响应帧：[地址][01][字节数][数据…][CRC-L][CRC-H]
         *
         * 功能：读取指定数量的线圈状态，每个线圈占1位
         *------------------------------------------------------------------*/
        case FC_READ_COILS:
            // 检查数量范围（1-2000）
            if (quantity < 1 || quantity > MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);  // 非法数据值
                break;
            }
            // 检查地址范围
            if (start_addr + quantity > MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);  // 非法数据地址
                break;
            }
            // 计算需要的字节数（向上取整到字节）
            byte_cnt = (unsigned char)((quantity + 7) / 8);
            // 构建响应帧
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_READ_COILS;
            tx_buf[2] = byte_cnt;  // 数据字节数
            // 从 modbus_coils 中提取对应位，按字节打包
            for (i = 0; i < byte_cnt; i++)
                tx_buf[3 + i] = (modbus_coils >> (start_addr + i * 8)) & 0xFF;
            SendResponse(3 + byte_cnt);  // 发送响应
            break;

        /*--------------------------------------------------------------------
         * FC=0x03  Read Holding Registers（读保持寄存器）
         * 请求帧：[地址][03][起始高][起始低][数量高][数量低][CRC-L][CRC-H]
         * 响应帧：[地址][03][字节数][数据高][数据低]…[CRC-L][CRC-H]
         *
         * 功能：读取指定数量的16位保持寄存器
         *------------------------------------------------------------------*/
        case FC_READ_HOLDING_REGS:
            // 检查数量范围（1-125）
            if (quantity < 1 || quantity > 125)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            // 检查地址范围
            if (start_addr + quantity > MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            // 计算数据字节数（每个寄存器2字节）
            byte_cnt = (unsigned char)(quantity * 2);
            // 构建响应帧
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_READ_HOLDING_REGS;
            tx_buf[2] = byte_cnt;
            // 逐个寄存器打包（大端格式）
            for (i = 0; i < quantity; i++)
            {
                tx_buf[3 + i * 2]     = (unsigned char)(modbus_regs[start_addr + i] >> 8);   // 高字节
                tx_buf[3 + i * 2 + 1] = (unsigned char)(modbus_regs[start_addr + i] & 0xFF); // 低字节
            }
            SendResponse(3 + byte_cnt);
            break;

        /*--------------------------------------------------------------------
         * FC=0x05  Write Single Coil（写单个线圈）
         * 请求帧：[地址][05][地址高][地址低][值高][值低][CRC-L][CRC-H]
         * 响应帧：原样回显请求帧
         *
         * 功能：设置单个线圈状态（0xFF00=ON, 0x0000=OFF）
         *------------------------------------------------------------------*/
        case FC_WRITE_SINGLE_COIL:
            // 检查地址范围
            if (start_addr >= MODBUS_COIL_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            // 解析线圈值（quantity 字段表示值）
            coil_val = (rx_buf[4] == 0xFF) ? 1 : 0;  // 0xFF00=ON, 其他=OFF
            // 更新线圈状态
            if (coil_val)
                modbus_coils |=  (1 << start_addr);  // 设置位
            else
                modbus_coils &= ~(1 << start_addr);  // 清除位

            // 原样回显请求帧（去掉CRC部分，SendResponse会重新加CRC）
            tx_buf[0] = rx_buf[0];
            tx_buf[1] = rx_buf[1];
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[3];
            tx_buf[4] = rx_buf[4];
            tx_buf[5] = rx_buf[5];
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * FC=0x06  Write Single Register（写单个寄存器）
         * 请求帧：[地址][06][地址高][地址低][值高][值低][CRC-L][CRC-H]
         * 响应帧：原样回显请求帧
         *
         * 功能：设置单个16位保持寄存器值
         *------------------------------------------------------------------*/
        case FC_WRITE_SINGLE_REG:
            // 检查地址范围
            if (start_addr >= MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            // 更新寄存器值（quantity 字段即为新值）
            modbus_regs[start_addr] = quantity;
            // 原样回显请求帧
            tx_buf[0] = rx_buf[0];
            tx_buf[1] = rx_buf[1];
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[3];
            tx_buf[4] = rx_buf[4];
            tx_buf[5] = rx_buf[5];
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * FC=0x10  Write Multiple Registers（写多个寄存器）
         * 请求帧：[地址][10][起始高][起始低][数量高][数量低][字节数][数据…][CRC]
         * 响应帧：[地址][10][起始高][起始低][数量高][数量低][CRC-L][CRC-H]
         *
         * 功能：设置多个连续的16位保持寄存器
         *------------------------------------------------------------------*/
        case FC_WRITE_MULTI_REGS:
            // 检查数量范围（1-123）
            if (quantity < 1 || quantity > 123)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            // 检查地址范围
            if (start_addr + quantity > MODBUS_REG_NUM)
            {
                SendException(func, EX_ILLEGAL_DATA_ADDR);
                break;
            }
            // 检查数据字节数是否匹配
            byte_cnt = rx_buf[6];  // 请求帧中的字节数字段
            if (byte_cnt != quantity * 2)
            {
                SendException(func, EX_ILLEGAL_DATA_VALUE);
                break;
            }
            // 更新多个寄存器（从 rx_buf[7] 开始是数据）
            for (i = 0; i < quantity; i++)
            {
                modbus_regs[start_addr + i] =
                    ((unsigned int)rx_buf[7 + i * 2] << 8) | rx_buf[7 + i * 2 + 1];  // 大端格式
            }
            // 响应帧：回显起始地址和数量
            tx_buf[0] = MODBUS_SLAVE_ADDR;
            tx_buf[1] = FC_WRITE_MULTI_REGS;
            tx_buf[2] = rx_buf[2];  // 起始地址高
            tx_buf[3] = rx_buf[3];  // 起始地址低
            tx_buf[4] = rx_buf[4];  // 数量高
            tx_buf[5] = rx_buf[5];  // 数量低
            SendResponse(6);
            break;

        /*--------------------------------------------------------------------
         * 不支持的功能码
         *------------------------------------------------------------------*/
        default:
            SendException(func, EX_ILLEGAL_FUNCTION);  // 非法功能码异常
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
 *
 * Modbus RTU 使用“3.5 字符时间”作为帧边界判定：
 *   当总线静默超过 3.5 字符时间（约 0.34ms @115200bps）时，认为一帧已经发送结束。
 *   本工程定时器以 1ms 为基准，因此取 MODBUS_TIMEOUT_MS = 2（约 2ms）作为安全超时。
 *
 * tick 逻辑：
 *   - 每收到一个字节（Modbus_RxByte）就重置 timeout_cnt
 *   - 若在超时内不再收到字节，则认为本帧接收完毕，置 rx_done=1 由 Modbus_Poll 处理
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
