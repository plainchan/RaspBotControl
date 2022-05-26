#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "stdlib.h"
#include "math.h"

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


//OLED模式设置
//0:4线串行模式

		    						  
//-----------------OLED端口定义----------------  
#define OLED_SCLK PAout(8)         //oled_SCL
#define OLED_SDIN PBout(15)        //oled_SDA
#define OLED_RST  PBout(14)        //oled_RES  高电平有效,低电平后必须重新初始化
#define OLED_RS   PBout(13)        //oled_DC
#define OLED_CS   PBout(12)        //oled_CS
	
		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
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

//图片 阴码 顺向 逐行
extern const unsigned char LOW_BATTERY[144];
extern const unsigned char BATTERY_4[36];
extern const unsigned char BATTERY_3[36];
extern const unsigned char BATTERY_2[36];
extern const unsigned char BATTERY_1[36];
extern const unsigned char BATTERY_0[36];
extern const unsigned char start_bmp[1024];
#endif  
	 



