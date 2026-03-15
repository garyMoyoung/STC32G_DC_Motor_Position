/*==============================================================================
 * spi.c  —  SPI 驱动（DMA 模式）
 *
 * 中断机制说明：
 *   SPI DMA 真实中断号 = 49，向量地址 = 018BH
 *   Keil C251 只支持 0~31，借用保留中断号 13（向量 006BH）
 *   需要 isr.asm 在 018BH 处写一条 LJMP 006BH 做向量跳转
 *   spi.c 中用 interrupt 13 定义中断函数（放在 006BH）
 *============================================================================*/

#include "spi.h"
#include <STC32G.H>
#include <intrins.h>

/*=============================================================================
 * DMA 全局变量
 *===========================================================================*/
#if (SPI_DMA_CFG == 1)
bit           SpiDmaFlag;
unsigned char xdata SpiDmaTxBuffer[SPI_DMA_BUF_SIZE];
#endif

/*=============================================================================
 * SPI 初始化
 *===========================================================================*/
void SPI_Init(void)
{
    /* SPI 基础配置 */
    P_SW1 = (P_SW1 & ~(3<<2)) | (1<<2);
    SSIG  = 1;
    SPEN  = 1;
    DORD  = 0;
    MSTR  = 1;
    CPOL  = 0;
    CPHA  = 0;
    SPCTL = (SPCTL & ~3) | 3;
    SPI_SCK  = 0;
    SPI_MOSI = 1;
    SPIF = 1;
    WCOL = 1;

#if (SPI_DMA_CFG == 1)
    P_SW2 |= 0x80;              // 打开扩展SFR

    DMA_SPI_STA  = 0x00;        // 清状态
    DMA_SPI_CFG  = 0xE0;        // SPIIE=1(中断使能), ACT_TX=1, ACT_RX=1
    DMA_SPI_AMT  = 0x00;

    /* 固定发送缓冲区地址，只设一次 */
    DMA_SPI_TXAH = (unsigned char)((unsigned int)SpiDmaTxBuffer >> 8);
    DMA_SPI_TXAL = (unsigned char)((unsigned int)SpiDmaTxBuffer & 0xFF);

    DMA_SPI_CFG2 = 0x00;
    DMA_SPI_CR   = 0x81;        // ENSPI=1, CLRFIFO=1（不触发）

    P_SW2 &= ~0x80;

    SpiDmaFlag = 0;

    EA = 1;                     // 确保全局中断开启
#endif
}

/*=============================================================================
 * 单字节阻塞发送
 *===========================================================================*/
void SPI_WriteByte(unsigned char dat)
{
    SPDAT = dat;
    while (SPIF == 0);      // 等移位完成（SPIF置位时SCK可能还在最后半周期）
    SPIF = 1;
    WCOL = 1;
    while (SPI_SCK);        // 等SCK回到空闲低电平，确保最后一位彻底移出
}

/*=============================================================================
 * 多字节发送
 *===========================================================================*/
void SPI_Transmit(unsigned char *buf, unsigned int len)
{
#if (SPI_DMA_CFG == 1)
    if (len == 0) return;
    while (SpiDmaFlag);

    SpiDmaFlag = 1;

    P_SW2 |= 0x80;
    /* 每次发送直接用原始缓冲区地址，不拷贝，避免地址错误 */
    DMA_SPI_TXAH = (unsigned char)((unsigned int)buf >> 8);
    DMA_SPI_TXAL = (unsigned char)((unsigned int)buf & 0xFF);
    DMA_SPI_AMT  = (unsigned char)(len - 1);
    DMA_SPI_CR  |= 0x40;
    P_SW2 &= ~0x80;
#else
    unsigned int i;
    for (i = 0; i < len; i++)
    {
        SPDAT = buf[i];
        while (SPIF == 0);
        SPIF = 1;
        WCOL = 1;
    }
#endif
}

/*=============================================================================
 * 等待DMA完成
 *===========================================================================*/
void SPI_DMA_Wait(void)
{
#if (SPI_DMA_CFG == 1)
    while (SpiDmaFlag);
#endif
}

/*=============================================================================
 * SPI DMA 中断服务函数
 *
 * 使用 interrupt 13（向量006BH，保留中断）
 * 配合 isr.asm：在018BH处 LJMP 006BH，将硬件中断导向此函数
 *===========================================================================*/
#if (SPI_DMA_CFG == 1)
void SPI_DMA_ISR(void) interrupt 13
{
    P_SW2 |= 0x80;
    DMA_SPI_STA = 0x00;         // 清中断标志
    P_SW2 &= ~0x80;

    SpiDmaFlag = 0;             // 通知上层完成
}
#endif
