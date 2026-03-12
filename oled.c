#include "oled.h"

#include "font.h"

#include "spi.h"

#include <STDIO.H>

#include <STC32G.H>

#include <STRING.H>



unsigned char xdata ShowBUFF[8][128];                           //OLEDȫ�ֻ���



//========================================================================

// ��������: delay

// ��������: OLED�����õ���ʱ

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void delay(void)

{

    int xdata i;

    

    for (i=0; i<100; i++);

}



//========================================================================

// ��������: delay_ms

// ��������: ms��ʱ����

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void delay_ms(unsigned int ms)

{

     unsigned int xdata i;

     do{

          i = MAIN_Fosc / 6000;

          while(--i);   //6T per loop

     }while(--ms);

}



//========================================================================

// ��������: OLED_WR_Byte

// ��������: OLED��ַ/����д�뺯��

// ��ڲ���: @dat:����    @cmd:����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_WR_Byte(unsigned char dat,unsigned char cmd)

{

    unsigned char udat[1] ;

    udat[0] = dat;

    if(cmd)

    {

        OLED_DC = 1;

    }

    else

    {

        OLED_DC = 0;

    }

    OLED_CS = 0;

    SPI_WriteByte(dat);

    delay();

    OLED_CS = 1;

    OLED_DC = 1;

}



//========================================================================

// ��������: OLED_Init

// ��������: OLED��ʼ������

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_Init(void)

{

    SPI_init();

    delay_ms(100);

    OLED_REST = 1;

    delay_ms(200);

    OLED_REST=0;//��λ

    delay_ms(200);

    OLED_REST = 1;



    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel

    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address

    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address

    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)

    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register

    OLED_WR_Byte(0xCF,OLED_CMD);// Set SEG Output Current Brightness

    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����

    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����

    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display

    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)

    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty

    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)

    OLED_WR_Byte(0x00,OLED_CMD);//-not offset

    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency

    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec

    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period

    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock

    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration

    OLED_WR_Byte(0x12,OLED_CMD);

    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh

    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level

    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)

    OLED_WR_Byte(0x00,OLED_CMD);//

    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable

    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable

    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)

    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 

    OLED_WR_Byte(0xAF,OLED_CMD);

    

    OLED_DisplayOn();

}





//========================================================================

// ��������: OLED_Set_Pos

// ��������: OLED������ʾλ��

// ��ڲ���: @x:x����     @y:y����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_Set_Pos(unsigned char x, unsigned char y) 

{ 

	OLED_WR_Byte((unsigned char)(0xb0+y),OLED_CMD);

	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);

	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD); 

}   





//========================================================================

// ��������: OLED_DrawBMP

// ��������: OLED��ʾͼƬ

// ��ڲ���: @x0:x���  @y0:y���    @x1:x�յ�   @y1:y�յ�  @BMP����ʾ����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])

{ 	

    unsigned int xdata j=0;

    unsigned char xdata x,y;



    for(y=y0;y<(y1+y0);y++)

    {

        OLED_Set_Pos(x0,y);

        for(x=0;x<x1;x++)

        {      

            OLED_WR_Byte(BMP[j++],OLED_DATA);	    	

        }

    }

} 



//========================================================================

// ��������: OLED_DisplayOn

// ��������: OLED����ʾ

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_DisplayOn(void)     //����ʾ

{

    OLED_WR_Byte(0x8D,OLED_CMD);//��ɱ�ʹ��

    OLED_WR_Byte(0x14,OLED_CMD);//������ɱ�

    OLED_WR_Byte(0xAF,OLED_CMD);//������Ļ       

}

  

//========================================================================

// ��������: OLED_DisplayOff

// ��������: OLED����ʾ

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_DisplayOff(void)     //����ʾ

{

    OLED_WR_Byte(0x8D,OLED_CMD);//��ɱ�ʹ��

    OLED_WR_Byte(0x10,OLED_CMD);//�رյ�ɱ�

    OLED_WR_Byte(0xAF,OLED_CMD);//�ر���Ļ        

}



//========================================================================

// ��������: OLED_LightSet

// ��������: OLED��������

// ��ڲ���: @num�� 0-255

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_LightSet(unsigned char num)     //��������

{

    OLED_WR_Byte(0x81,OLED_CMD);//

    OLED_WR_Byte(num,OLED_CMD);//  

    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh

    OLED_WR_Byte(0x20,OLED_CMD);//Set VCOM Deselect Level   

}





//========================================================================

// ��������: OLED_BuffClear

// ��������: OLED��ջ���

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffClear(void)     //����

{

    memset(ShowBUFF,0,128*8);

}



//========================================================================

// ��������: OLED_BuffShow

// ��������: OLEDˢ����ʾ

// ��ڲ���: ��

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2024 - 4 - 3

// ��ǰ����: ZCF, PatZer0

// ������ע:

//========================================================================

void OLED_BuffShow(void)     

{

    unsigned char xdata i,n,j;

    for(i=0;i<8;i++)

    {

        j = 0xb0 + i;             //C251�����Ͻ�������д�Ա���warning

        OLED_WR_Byte(j,OLED_CMD); //��������ʼ��ַ

        OLED_WR_Byte(0x00,OLED_CMD);   //���õ�����ʼ��ַ    // �޸ĵ�����ʼ�������������Ļ��ʾ��ȫ

        OLED_WR_Byte(0x10,OLED_CMD);   //���ø�����ʼ��ַ



        for(n=0;n<128;n++)

            OLED_WR_Byte(ShowBUFF[i][n],OLED_DATA);

    }

}



//========================================================================

// ��������: OLED_BuffShowPoint

// ��������: OLED��ʾһ����

// ��ڲ���: @x��x���   @y:y���

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowPoint(unsigned char x,unsigned char y)     //OLED��ʾһ����

{

    ShowBUFF[y/8][x] |= 1<<(y%8);

}



//========================================================================

// ��������: OLED_BuffShowPoint

// ��������: OLED��ʾһ����

// ��ڲ���: @x1��x���  @y1��y���  @x2��x�յ�   @y2��y�յ�

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowLine( unsigned char x1, unsigned char y1, unsigned char x2,unsigned char y2)

{

    unsigned char x,y;

    if( x1>x2 )

    {

        x=x1;x1=x2;x2=x;

        y=y1;y1=y2;y2=y;

    }

    if(x1!=x2)

    {

        for( x = x1; x <= x2; x++ )

        {

            if( y2>y1 )

                OLED_BuffShowPoint(x, (unsigned char)(y1+(unsigned int)(y2-y1)*(unsigned int)x/(unsigned int)(x2-x1)));

            else

                OLED_BuffShowPoint(x, (unsigned char)(y1-(unsigned int)(y1-y2)*(unsigned int)x/(unsigned int)(x2-x1)));

        }        

    }

    else

    {

        if( y1>y2 )

        {

            for( y = y2; y <= y1; y++ )

               OLED_BuffShowPoint(x1, y); 

        }

        else

        {

            for( y = y1; y <= y2; y++ )

               OLED_BuffShowPoint(x1, y);             

        }

    }

}



//========================================================================

// ��������: OLED_BuffShowRectangle

// ��������: OLED��ʾһ������

// ��ڲ���: @x1��x���  @y1��y���  @x2��x�յ�   @y2��y�յ�

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowRectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)

{

    OLED_BuffShowLine(x1, y1, x2, y1);

    OLED_BuffShowLine(x1, y1, x1, y2);

    OLED_BuffShowLine(x1, y2, x2, y2);

    OLED_BuffShowLine(x2, y1, x2, y2);

}



//========================================================================

// ��������: OLED_BuffShowCircle

// ��������: OLED��ʾһ��Բ��

// ��ڲ���: @x��x��  @y��y��  @r:�뾶

// ��������: ��

// ��ǰ�汾: VER1.1

// �޸�����: 2024 - 3 - 28

// ��ǰ����: ZCF, PatZer0

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowCircle(unsigned char x, unsigned char y, unsigned char r)

{

    int a, b;

    int di;

    unsigned char x_add_a, x_add_b, x_sub_a, x_sub_b, y_add_a, y_add_b, y_sub_a, y_sub_b;



    a = 0;

    b = r;

    di = 3 - (r << 1);       //�ж��¸���λ�õı�־

 

    while (a <= b)

    {

        x_add_a = x + a;

        x_add_b = x + b;

        x_sub_a = x - a;

        x_sub_b = x - b;

        y_add_a = y + a;

        y_add_b = y + b;

        y_sub_a = y - a;

        y_sub_b = y - b;

        OLED_BuffShowPoint(x_add_a, y_sub_b);        //5

        OLED_BuffShowPoint(x_add_b, y_sub_a);        //0

        OLED_BuffShowPoint(x_sub_b, y_add_a);        //4

        OLED_BuffShowPoint(x_sub_a, y_add_b);        //6

        OLED_BuffShowPoint(x_sub_a, y_sub_b);        //1

        OLED_BuffShowPoint(x_add_b, y_add_a);

        OLED_BuffShowPoint(x_sub_b, y_sub_a);        //2

        OLED_BuffShowPoint(x_add_a, y_add_b);        //7

        //��д���룬����C251��Warning



        a++;

 

        //ʹ��Bresenham�㷨��Բ

        if (di < 0)di += 4 * a + 6;

        else

        {

            di += 10 + 4 * (a - b);

            b--;

        }

    }

}



//========================================================================

// ��������: OLED_BuffShowChar

// ��������: OLED��ʾһ��8*16���ַ�

// ��ڲ���: @x��x��  @y��y��  @asc:�ַ�   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowChar(unsigned char x,unsigned char y,char asc, unsigned char mode)     //OLED��ʾһ��8*16�ַ�

{

    unsigned char j,k;

    for( j=0;j<2;j++ )

    {

        for( k=0;k<8;k++ )

        {

            if(mode==0)            

                ShowBUFF[j+y][x+k]=Ascll_16[(asc-' ')*2][j*8+k];

            else

                ShowBUFF[j+y][x+k]=~Ascll_16[(asc-' ')*2][j*8+k];

        }

    }    

}



//========================================================================

// ��������: OLED_BuffShowGBK

// ��������: OLED��ʾ����

// ��ڲ���: @x��x��  @y��y��  @gbk:����   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.1

// �޸�����: 2024 - 3 - 29

// ��ǰ����: ZCF, PatZer0

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//          �õ��ĺ�����Ҫ�Լ�ȡģ�ŵ��ֿ���ֿ��Ļ����������ѭ������Ҫ�Ŵ�

//          �ֿ��ļ�font.h���к��ֵ�GBK16���飬�����Լ��޸�

//          ԭ����ʹ�ù̶��ı�������������ֱ�Ӹ�ΪGBK16�����С��ȷ�������ֿⶼ������

//========================================================================

void OLED_BuffShowGBK(unsigned char x,unsigned char y,char *gbk,unsigned char mode)     //OLED��ʾһ��16*16����

{

    unsigned char i;

    unsigned char j,k;

    for( i=0;i<sizeof(GBK16)/sizeof(GBK16[0]);i++ )

    {

        if((gbk[0] == GBK16[i].gbn_bum[0])

         &&(gbk[1] == GBK16[i].gbn_bum[1])

			)

        {

            for( j=0;j<2;j++ )

            {

                for( k=0;k<16;k++ )

                {

                    if( mode ==0 )

                        ShowBUFF[j+y][x+k]=GBK16[i].gbk_font[j*16+k];

                    else

                        ShowBUFF[j+y][x+k]=~GBK16[i].gbk_font[j*16+k];

                }

            }

            break;

        }

    }    

}



//========================================================================

// ��������: OLED_BuffShowString

// ��������: OLED��ʾ�ַ����������ַ�����

// ��ڲ���: @x��x��  @y��y��  @s�ַ���   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//          �õ��ĺ�����Ҫ�Լ�ȡģ�ŵ��ֿ���ֿ��Ļ����������ѭ������Ҫ�Ŵ�

//========================================================================

void OLED_BuffShowString(unsigned char x,unsigned char y,char *s,unsigned char mode)

{

    char s_num[2];

	while(*s != '\0')       //�ַ�����Ϊ�գ�ѭ��

	{

        if ((unsigned char)*s < 0x80)     //�����������ݵĴ�С�ж����ַ����Ǻ��֣�

        {

            OLED_BuffShowChar(x,y,*s,mode);

            x+=8;

            s++;

        }

        else

        {

            s_num[0] = *s ;

            s_num[1] = *(s+1) ;

            OLED_BuffShowGBK(x,y,s_num,mode);

            x+=16;

            s+=2;

        }

		if(x>127)

        {

            x=0;

            y+=2;

        }

	}       

}



//========================================================================

// ��������: OLED_BuffShowNum

// ��������: OLED��ʾ���α���

// ��ڲ���: @x��x��  @y��y��  @num�����α���   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.1

// �޸�����: 2024 - 3 - 28

// ��ǰ����: ZCF, PatZer0

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowNum(unsigned char x,unsigned char y,long num,unsigned char mode)

{

    unsigned char xdata str[10];

    memset(str,0,10);

    sprintf(str,"%ld",num);

    OLED_BuffShowString(x,y,(unsigned char*)str, mode);

}



//========================================================================

// ��������: OLED_BuffShowNum02F

// ��������: OLED��ʾһ����λС������

// ��ڲ���: @x��x��  @y��y��  @num��С������   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//========================================================================

void OLED_BuffShowNum02F(unsigned char x,unsigned char y,float num,unsigned char mode)

{

    unsigned char xdata str[10];

    memset(str,0,10);

    sprintf(str,"%.2f",num);

    OLED_BuffShowString(x,y,(unsigned char*)str,mode);

}





//========================================================================

// ��������: OLED_BuffShowChar32

// ��������: OLED��ʾ16*32���ַ�

// ��ڲ���: @x��x��  @y��y��  @num����ʾ�ڼ����ַ�   @mode��0����  1����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע: д����ǻ��棬��Ҫ�����OLED_BuffShow������ʾ����

//          �õ�����Ҫ�Լ�ȡģŶ

//========================================================================

void OLED_BuffShowChar32(unsigned char  x,unsigned char  y,unsigned char num, unsigned char mode)     

{

    unsigned char j,k;

    for( j=0;j<4;j++ )

    {

        for( k=0;k<16;k++ )

        {

            if( mode==0 )

                ShowBUFF[j+y][x+k]=lib_num1632[num][j*16+k];

            else

                ShowBUFF[j+y][x+k]=~lib_num1632[num][j*16+k];

        }

    }            

}





//========================================================================

// ��������: OLED_BuffShowBMP

// ��������: OLED����д��ͼƬ

// ��ڲ���: @x0:x���  @y0:y���    @x1:x�յ�   @y1:y�յ�  @BMP����ʾ����

// ��������: ��

// ��ǰ�汾: VER1.0

// �޸�����: 2023 - 6 - 7

// ��ǰ����: ZCF

// ������ע:

//========================================================================

void OLED_BuffShowBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])

{ 	

    unsigned int xdata num=0;

    unsigned char k,j;

    

    for( j=0;j<y1;j++ )

    {

        for( k=0;k<x1;k++ )

        {

            ShowBUFF[j+y0][x0+k]=BMP[num++];

        }

    }  

} 

 