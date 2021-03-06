#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//SSD1306 OLED 驱动IC驱动代码
//驱动方式:8080并口/4线串口
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  


//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 		   
u8 OLED_GRAM[128][8];	 

//更新显存到LCD		 
void oled_update(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//向SSD1306写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	OLED_RS=cmd; //写命令 
	OLED_CS=0;		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK=0;
		if(dat&0x80)OLED_SDIN=1;
		else OLED_SDIN=0;
		OLED_SCLK=1;
		dat<<=1;   
	}				 
	OLED_CS=1;		  
	OLED_RS=1;   	  
} 

	  	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	oled_update();//更新显示
}
//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void oled_point(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,清空;1,填充	  
void oled_fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)oled_point(x,y,dot);
	}													    
	oled_update();//更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void oled_char(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数
	chr=chr-' ';//得到偏移后的值		 
    for(t=0;t<csize;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[chr][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[chr][t];	//调用2412字体
		else return;								//没有的字库
    for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)oled_point(x,y,mode);
			else oled_point(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n函数
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void oled_num(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				oled_char(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	oled_char(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//显示字符串
//x,y:起点坐标  
//size:字体大小 
//*p:字符串起始地址 
void oled_str(u8 x,u8 y,const u8 *p,u8 size)
{	
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(64-size)){y=x=0;OLED_Clear();}
        oled_char(x,y,*p,size,1);	 
        x+=size/2;
        p++;
    }  
	
}	
//初始化SSD1306					    
void oled_init(void)
{ 	 				 	 					    
	GPIO_InitTypeDef  GPIO_InitStructure;
  	  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE );
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 	GPIO_SetBits(GPIOA,GPIO_Pin_8);	
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	 
	GPIO_ResetBits(GPIOB,GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12);
 	GPIO_SetBits(GPIOB,GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12);	
							  
 		  
	OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示
	OLED_WR_Byte(0xD5,OLED_CMD); //设置时钟分频因子,震荡频率
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xA8,OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0X3F,OLED_CMD); //默认0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0X00,OLED_CMD); //默认为0

	OLED_WR_Byte(0x40,OLED_CMD); //设置显示开始行 [5:0],行数.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); //bit2，开启/关闭
	OLED_WR_Byte(0x20,OLED_CMD); //设置内存地址模式
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1,OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA,OLED_CMD); //设置COM硬件引脚配置
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]配置
		 
	OLED_WR_Byte(0x81,OLED_CMD); //对比度设置
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //设置VCOMH 电压倍率
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6,OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //开启显示	 
	OLED_Clear();
}  




/*
*      name  :    oled_print(unsigned char x,unsigned char y,unsigned char ch[])
*      size  :    汉字16x16,英文16x8
*   function :    显示汉字,英文混合字符串
*   example  :    oled_print(0,0,"我爱你ILY")
*   parameter:    显示的位置（x,y），要显示的字符串
*   notice   :
*/
void oled_print(unsigned char x,unsigned char y,u8 ch[])
{
	
  u8 temp,t,t1;
  u8 y0=y;
  u8 wm,ii = 0;
  u8 ch2[2];
	
  while(ch[ii] != '\0')
	{
    wm = 0;
    if(ch[ii] < 128&&(ch[ii]<='~')&&(ch[ii]>=' '))      //不是汉字，且不是非法英文字符
		{
			ch2[0] = ch[ii];
			ch2[1] = '\0';			//字母占一个字节
			oled_str(x,y,ch2,16);	//显示字母16x16
			x+=8;
			if(x>127)
			{
				x=0;
				y+=16;
			}

			ii += 1;
		}
		else
		{
			for(wm=0;wm<Idx_num-1;wm++)
			{
				if(F16x16_Idx[wm] == ch[ii]&&F16x16_Idx[wm + 1] == ch[ii + 1])             //一个汉字占两个字节
				{
					for(t=0; t<32; t++)
					{
						temp= F16x16[wm*16+t];
						for(t1=0; t1<8; t1++)
						{
							if(temp&0x80)oled_point(x,y,1);
							else oled_point(x,y,0);
							temp<<=1;
							y++;
							if((y-y0) == 16)
							{		
								x++;
								if(x==128)
								{
									x=0;
									y0+=16;
								}
								y=y0;
								break;
							}
						}

					}
				}
			}
		 ii +=2;
		}
	}
}

void oled_digit(u8 x,u8 y,int num,u8 size)
{
	static u8 last_x =0;
	u8 interval=size/2;
	
	if(num < 0) 
	{
		oled_char(x,y,'-',size,1);
		x+=interval;
	}
	
	unsigned  int integer = abs(num);
	unsigned  int  base =1;
	while(base <= integer/10)
	{
		base*=10;
	}
	while(base>0)
	{
		u8 ch = integer/base+'0';
		oled_char(x,y,ch,size,1);
		integer = integer-base*(ch-'0');
		base/=10;
		x+=interval;
	}
	for(int i=x;i<last_x;i+=interval)
		oled_char(i,y,' ',size,1);
	last_x = x+(num>0?0:interval);
}

void oled_float(u8 x,u8 y,float num,u8 precison,u8 size)
{
	static u8 last_x =0;
	u8 interval=size/2;
	
	if(num < 0) 
	{
		oled_char(x,y,'-',size,1);
		x+=interval;
	}
	
	u32 integer = fabs(num);
	u32 base = mypow(10,precison);
	base=1;
	while(base <= integer/10)
	{
		base*=10;
	}
	while(base>0)
	{
		u8 ch = integer/base+'0';
		oled_char(x,y,ch,size,1);
		integer = integer-base*(ch-'0');
		base/=10;
		x+=interval;
	}
	oled_char(x,y,'.',size,1);
	x+=interval;
	
	integer = fabs(num);
	for(int i=1;i<=precison;++i)
	{
		integer*=10;
		u8 ch = (u32)(fabs(num)*mypow(10,i)) - integer+'0';
		oled_char(x,y,ch,size,1);
		integer+=ch-'0';
		x+=interval;
	}
	for(int i=x;i<last_x;i+=interval)
		oled_char(i,y,' ',size,1);
	last_x = x+(num>0?0:interval);
}

/*
*      name  :    oled_picture(u8 x,u8 y,u8 w,u8 h,u8 bmp[])
*   function :    显示图片
*   parameter:    显示的位置（x,y），图片大小(w,h)
*   example  :    oled_picture(0,0,128,64,pic)
*   notice   :    数据逐行存放，高位在前,索引不能超出bmp的size!!!
*/
void oled_picture(u8 x,u8 y,u8 w,u8 h,const unsigned char *bmp)
{
	u8 pos_x=x,pos_y=y;
	u8 x1=x+w;
	u16 size = w*h/8; 
	for(int n=0;n<size;++n)
	{   
		u8 p = bmp[n];
		for(int i=0;i<8;++i)
		{
			if(p&0x80)oled_point(pos_x,pos_y,1);else oled_point(pos_x,pos_y,0);
			p<<=1;
			++pos_x;
			if(pos_x==x1)
			{
				pos_x=x;
				++pos_y;
			}
		}
	}  	 
      	
	
}























