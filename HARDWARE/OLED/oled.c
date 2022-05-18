#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SSD1306 OLED ����IC��������
//������ʽ:8080����/4�ߴ���
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  


//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 		   
u8 OLED_GRAM[128][8];	 

//�����Դ浽LCD		 
void oled_update(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//��SSD1306д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	OLED_RS=cmd; //д���� 
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

	  	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	oled_update();//������ʾ
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void oled_point(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
//x1,y1,x2,y2 �������ĶԽ�����
//ȷ��x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,���;1,���	  
void oled_fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)oled_point(x,y,dot);
	}													    
	oled_update();//������ʾ
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void oled_char(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ		 
    for(t=0;t<csize;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[chr][t];	//����1608����
		else if(size==24)temp=asc2_2412[chr][t];	//����2412����
		else return;								//û�е��ֿ�
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
//m^n����
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
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
//��ʾ�ַ���
//x,y:�������  
//size:�����С 
//*p:�ַ�����ʼ��ַ 
void oled_str(u8 x,u8 y,const u8 *p,u8 size)
{	
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(64-size)){y=x=0;OLED_Clear();}
        oled_char(x,y,*p,size,1);	 
        x+=size/2;
        p++;
    }  
	
}	
//��ʼ��SSD1306					    
void oled_init(void)
{ 	 				 	 					    
	GPIO_InitTypeDef  GPIO_InitStructure;
  	  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE );
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
 	GPIO_SetBits(GPIOA,GPIO_Pin_8);	
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	 
 	GPIO_SetBits(GPIOB,GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12);	
							  
 		  
	OLED_WR_Byte(0xAE,OLED_CMD); //�ر���ʾ
	OLED_WR_Byte(0xD5,OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Byte(0xA8,OLED_CMD); //��������·��
	OLED_WR_Byte(0X3F,OLED_CMD); //Ĭ��0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //������ʾƫ��
	OLED_WR_Byte(0X00,OLED_CMD); //Ĭ��Ϊ0

	OLED_WR_Byte(0x40,OLED_CMD); //������ʾ��ʼ�� [5:0],����.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //��ɱ�����
	OLED_WR_Byte(0x14,OLED_CMD); //bit2������/�ر�
	OLED_WR_Byte(0x20,OLED_CMD); //�����ڴ��ַģʽ
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1,OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA,OLED_CMD); //����COMӲ����������
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]����
		 
	OLED_WR_Byte(0x81,OLED_CMD); //�Աȶ�����
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9,OLED_CMD); //����Ԥ�������
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //����VCOMH ��ѹ����
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6,OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //������ʾ	 
	OLED_Clear();
}  




/*
*      name  :    oled_print(unsigned char x,unsigned char y,unsigned char ch[])
*      size  :    ����16x16,Ӣ��16x8
*   function :    ��ʾ����,Ӣ�Ļ���ַ���
*   example  :    oled_print(0,0,"�Ұ���ILY")
*   parameter:    ��ʾ��λ�ã�x,y����Ҫ��ʾ���ַ���
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
    if(ch[ii] < 128&&(ch[ii]<='~')&&(ch[ii]>=' '))      //���Ǻ��֣��Ҳ��ǷǷ�Ӣ���ַ�
		{
			ch2[0] = ch[ii];
			ch2[1] = '\0';			//��ĸռһ���ֽ�
			oled_str(x,y,ch2,16);	//��ʾ��ĸ16x16
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
				if(F16x16_Idx[wm] == ch[ii]&&F16x16_Idx[wm + 1] == ch[ii + 1])             //һ������ռ�����ֽ�
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


void oled_float(u8 x,u8 y,float num,u8 precison,u8 size)
{
	u32 integer = num;
	u16 base = mypow(10,precison);
	u32 decimal = num*base-integer*base;
	base=1;
	while(base <= integer/10)
	{
		base*=10;
	}
	while(base>0)
	{
		char ch = integer/base+'0';
		oled_char(x,y,ch,size,1);
		integer = integer-base*(ch-'0');
		base/=10;
		x+=size/2;
	}
	oled_char(x,y,'.',size,1);
	x+=size/2;
	base=1;
	while(base <= decimal/10)
	{
		base*=10;
	}
	while(base>0)
	{
		char ch = decimal/base+'0';
		oled_char(x,y,ch,size,1);
		decimal=decimal-base*(ch-'0');
		base/=10;
		x+=size/2;
	}
}

/*
*      name  :    oled_picture(u8 x,u8 y,u8 w,u8 h,u8 bmp[])
*   function :    ��ʾͼƬ
*   parameter:    ��ʾ��λ�ã�x,y����ͼƬ��С(w,h)
*   example  :    oled_picture(0,0,128,64,pic)
*   notice   :    �������д�ţ���λ��ǰ,�������ܳ���bmp��size!!!
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
			if(p&0x80)oled_point(pos_x,pos_y,1);
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























