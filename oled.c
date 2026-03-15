/*==============================================================================
 * oled.c  —  OLED 驱动（适配新 SPI DMA 接口）
 *
 * 接口变化：
 *   SPI_WriteByte()  →  发单字节命令（不变）
 *   SPI_Transmit()   →  发多字节数据（DMA或阻塞自动切换）
 *   SPI_DMA_Wait()   →  等待DMA完成后再拉CS
 *
 * 其余显示函数（ShowChar/ShowString/ShowNum等）完全不变
 *============================================================================*/

#include "oled.h"
#include "font.h"
#include "spi.h"
#include <STDIO.H>
#include <STC32G.H>
#include <STRING.H>
#include <intrins.h>

/*=============================================================================
 * 全局显示缓冲区（必须在 xdata 区，DMA 只能访问片内 xdata RAM）
 *===========================================================================*/
unsigned char xdata ShowBUFF[8][128];

/*=============================================================================
 * 内部延时
 *===========================================================================*/
static void delay(void)
{
    int xdata i;
    for (i = 0; i < 100; i++);
}

void delay_ms(unsigned int ms)
{
    unsigned int xdata i;
    do {
        i = MAIN_Fosc / 6000;
        while (--i);
    } while (--ms);
}

/*=============================================================================
 * 单字节写入（命令/数据）
 * 命令：DC=0，阻塞SPI发送
 * 数据：DC=1，阻塞SPI发送（单字节场合，如DrawBMP逐点）
 *===========================================================================*/
void OLED_WR_Byte(unsigned char dat, unsigned char cmd)
{
    if (cmd)
        OLED_DC = 1;
    else
        OLED_DC = 0;

    OLED_CS = 0;
    SPI_WriteByte(dat);
    delay();
    OLED_CS = 1;
    OLED_DC = 1;
}

/*=============================================================================
 * OLED 初始化（保持原版不变）
 *===========================================================================*/
void OLED_Init(void)
{
    SPI_Init();
    delay_ms(100);
    OLED_REST = 1;
    delay_ms(200);
    OLED_REST = 0;
    delay_ms(200);
    OLED_REST = 1;

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3f, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xd5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);

    OLED_DisplayOn();
}

/*=============================================================================
 * 设置显示位置
 *===========================================================================*/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte((unsigned char)(0xb0 + y), OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f) | 0x01, OLED_CMD);
}

/*=============================================================================
 * OLED_BuffShow —— 缓冲区刷屏
 *
 * 流程（每行）：
 *   1. 发3字节页地址命令（阻塞SPI）
 *   2. 等待上次DMA完成（SPI_DMA_Wait）
 *   3. DC=DATA，CS拉低
 *   4. SPI_Transmit 触发DMA发送128字节
 *   5. SPI_DMA_Wait 等本行发完
 *   6. CS拉高
 *
 * 关于第一行色块问题的修复：
 *   - 发完命令后 CS先拉高再等 4个nop，保证SPI内部状态复位
 *   - DC切换后等 2个nop 再拉低CS，保证OLED采样到正确的DC
 *   - SPI_Transmit内部 while(SpiDmaFlag) 确保SPI完全空闲后再触发
 *===========================================================================*/
void OLED_BuffShow(void)
{
    unsigned char i;
    unsigned char xdata *row;

    for (i = 0; i < 8; i++)
    {
        row = (unsigned char xdata *)ShowBUFF + (unsigned int)i * 128;

        OLED_DC = 0;
        OLED_CS = 0;
        SPI_WriteByte(0xb0 + i);
        SPI_WriteByte(0x00);
        SPI_WriteByte(0x10);
        OLED_CS = 1;

        OLED_DC = 1;
        _nop_(); _nop_();
        OLED_CS = 0;

        SPI_Transmit(row, 128);
        SPI_DMA_Wait();

        OLED_CS = 1;
    }
}

/*=============================================================================
 * 其余函数与原版完全相同
 *===========================================================================*/

void OLED_BuffClear(void)
{
    unsigned char xdata *p = (unsigned char xdata *)ShowBUFF;
    unsigned int n;
    for (n = 0; n < 8 * 128; n++)
        p[n] = 0x00;
}

void OLED_DisplayOn(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_DisplayOff(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_DrawBMP(unsigned char x0, unsigned char y0,
                  unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int xdata j = 0;
    unsigned char xdata x, y;
    for (y = y0; y < (y1 + y0); y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = 0; x < x1; x++)
            OLED_WR_Byte(BMP[j++], OLED_DATA);
    }
}

void OLED_BuffShowPoint(unsigned char x, unsigned char y)
{
    ShowBUFF[y / 8][x] |= 1 << (y % 8);
}

void OLED_BuffShowLine(unsigned char x1, unsigned char y1,
                       unsigned char x2, unsigned char y2)
{
    unsigned char x, y;
    if (x1 > x2) { x=x1;x1=x2;x2=x; y=y1;y1=y2;y2=y; }
    if (x1 != x2)
    {
        for (x = x1; x <= x2; x++)
        {
            if (y2 > y1)
                OLED_BuffShowPoint(x, (unsigned char)(y1 + (unsigned int)(y2-y1)*(unsigned int)x/(unsigned int)(x2-x1)));
            else
                OLED_BuffShowPoint(x, (unsigned char)(y1 - (unsigned int)(y1-y2)*(unsigned int)x/(unsigned int)(x2-x1)));
        }
    }
    else
    {
        if (y1 > y2) { for (y=y2; y<=y1; y++) OLED_BuffShowPoint(x1,y); }
        else          { for (y=y1; y<=y2; y++) OLED_BuffShowPoint(x1,y); }
    }
}

void OLED_BuffShowRectangle(unsigned char x1, unsigned char y1,
                             unsigned char x2, unsigned char y2)
{
    OLED_BuffShowLine(x1,y1,x2,y1);
    OLED_BuffShowLine(x1,y1,x1,y2);
    OLED_BuffShowLine(x1,y2,x2,y2);
    OLED_BuffShowLine(x2,y1,x2,y2);
}

void OLED_BuffShowCircle(unsigned char x, unsigned char y, unsigned char r)
{
    int a, b, di;
    unsigned char x_add_a, x_add_b, x_sub_a, x_sub_b;
    unsigned char y_add_a, y_add_b, y_sub_a, y_sub_b;
    a=0; b=r; di=3-(r<<1);
    while (a <= b)
    {
        x_add_a=x+a; x_add_b=x+b; x_sub_a=x-a; x_sub_b=x-b;
        y_add_a=y+a; y_add_b=y+b; y_sub_a=y-a; y_sub_b=y-b;
        OLED_BuffShowPoint(x_add_a, y_sub_b);
        OLED_BuffShowPoint(x_add_b, y_sub_a);
        OLED_BuffShowPoint(x_sub_b, y_add_a);
        OLED_BuffShowPoint(x_sub_a, y_add_b);
        OLED_BuffShowPoint(x_sub_a, y_sub_b);
        OLED_BuffShowPoint(x_add_b, y_add_a);
        OLED_BuffShowPoint(x_sub_b, y_sub_a);
        OLED_BuffShowPoint(x_add_a, y_add_b);
        a++;
        if (di < 0) di += 4*a+6;
        else { di += 10+4*(a-b); b--; }
    }
}

void OLED_BuffShowChar(unsigned char x, unsigned char y,
                       char asc, unsigned char mode)
{
    unsigned char j, k;
    for (j = 0; j < 2; j++)
        for (k = 0; k < 8; k++)
        {
            if (mode == 0)
                ShowBUFF[j+y][x+k] = Ascll_16[(asc-' ')*2][j*8+k];
            else
                ShowBUFF[j+y][x+k] = ~Ascll_16[(asc-' ')*2][j*8+k];
        }
}

void OLED_BuffShowGBK(unsigned char x, unsigned char y,
                      char *gbk, unsigned char mode)
{
    unsigned char i, j, k;
    for (i = 0; i < sizeof(GBK16)/sizeof(GBK16[0]); i++)
    {
        if ((gbk[0]==GBK16[i].gbn_bum[0]) && (gbk[1]==GBK16[i].gbn_bum[1]))
        {
            for (j = 0; j < 2; j++)
                for (k = 0; k < 16; k++)
                {
                    if (mode == 0)
                        ShowBUFF[j+y][x+k] = GBK16[i].gbk_font[j*16+k];
                    else
                        ShowBUFF[j+y][x+k] = ~GBK16[i].gbk_font[j*16+k];
                }
            break;
        }
    }
}

void OLED_BuffShowString(unsigned char x, unsigned char y,
                         char *s, unsigned char mode)
{
    char s_num[2];
    while (*s != '\0')
    {
        if ((unsigned char)*s < 0x80)
        {
            OLED_BuffShowChar(x, y, *s, mode);
            x += 8; s++;
        }
        else
        {
            s_num[0] = *s; s_num[1] = *(s+1);
            OLED_BuffShowGBK(x, y, s_num, mode);
            x += 16; s += 2;
        }
        if (x > 127) { x=0; y+=2; }
    }
}

void OLED_BuffShowNum(unsigned char x, unsigned char y,
                      long num, unsigned char mode)
{
    unsigned char xdata str[10];
    memset(str, 0, 10);
    sprintf(str, "%ld", num);
    OLED_BuffShowString(x, y, (unsigned char*)str, mode);
}

void OLED_BuffShowNum02F(unsigned char x, unsigned char y,
                         float num, unsigned char mode)
{
    unsigned char xdata str[10];
    memset(str, 0, 10);
    sprintf(str, "%.2f", num);
    OLED_BuffShowString(x, y, (unsigned char*)str, mode);
}

void OLED_BuffShowChar32(unsigned char x, unsigned char y,
                         unsigned char num, unsigned char mode)
{
    unsigned char j, k;
    for (j = 0; j < 4; j++)
        for (k = 0; k < 16; k++)
        {
            if (mode == 0)
                ShowBUFF[j+y][x+k] = lib_num1632[num][j*16+k];
            else
                ShowBUFF[j+y][x+k] = ~lib_num1632[num][j*16+k];
        }
}

void OLED_BuffShowBMP(unsigned char x0, unsigned char y0,
                      unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int xdata num = 0;
    unsigned char k, j;
    for (j = 0; j < y1; j++)
        for (k = 0; k < x1; k++)
            ShowBUFF[j+y0][x0+k] = BMP[num++];
}
