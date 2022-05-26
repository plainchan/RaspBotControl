#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "stdlib.h"
#include "math.h"

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


//OLEDģʽ����
//0:4�ߴ���ģʽ

		    						  
//-----------------OLED�˿ڶ���----------------  
#define OLED_SCLK PAout(8)         //oled_SCL
#define OLED_SDIN PBout(15)        //oled_SDA
#define OLED_RST  PBout(14)        //oled_RES  �ߵ�ƽ��Ч,�͵�ƽ��������³�ʼ��
#define OLED_RS   PBout(13)        //oled_DC
#define OLED_CS   PBout(12)        //oled_CS
	
		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����
//OLED�����ú���
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void oled_update(void);		   
							   		    
void oled_init(void);
void OLED_Clear(void);
void oled_point(u8 x,u8 y,u8 t);
void oled_fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void oled_char(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void oled_num(u8 x,u8 y,u32 num,u8 len,u8 size);
void oled_str(u8 x,u8 y,const u8 *p,u8 size);	 
void oled_print(unsigned char x,unsigned char y,unsigned char ch[]);
void oled_float(u8 x,u8 y,float num,u8 precison,u8 totalLen,u8 size);
void oled_picture(u8 x,u8 y,u8 w,u8 h,const unsigned char *bmp);
void oled_digit(u8 x,u8 y,long long num,u8 totalLen,u8 size);

//ͼƬ ���� ˳�� ����
extern const unsigned char LOW_BATTERY[144];
extern const unsigned char BATTERY_4[36];
extern const unsigned char BATTERY_3[36];
extern const unsigned char BATTERY_2[36];
extern const unsigned char BATTERY_1[36];
extern const unsigned char BATTERY_0[36];
extern const unsigned char start_bmp[1024];
#endif  
	 



