#include "stm32f10x_gpio.h"
#include "ps2lib.h"
#include "stdint.h"
#include "delay.h"
#include "oled.h"

//static uint8_t enter_config[]={0x01,0x43,0x00,0x01,0x00};
//static uint8_t set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
//static uint8_t set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
//static uint8_t exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
//static uint8_t enable_rumble[]={0x01,0x4D,0x00,0x00,0x01};
//static uint8_t type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};  // CMD,Find out what buttons are included in poll responses

uint8_t ps2_resp_data[21];

int a =0;
uint8_t sendCommmandAndGetResp(uint8_t byte)
{
	uint8_t res = 0;

	for(int i=0;i<8;++i)
	{
		//单片机向手柄发送命令
		if(byte&(1<<i) )CMD=1;
		else CMD = 0;             

		// 在时钟下降沿时，完成数据（1bit））的发送与接收


		CLK = 0;

		delay_us(5);

		if(DAT) res|=(1<<i);
		else	a++;
		oled_digit(60,0,a,3,12);
		
			
		
		CLK = 1;
		delay_us(5);
	}
	CMD = 1;
	delay_us(5);

	return res;
}

uint8_t  sendCommandLists(uint8_t *byte,uint8_t len)
{
	reset_resp_data();

	
	CS = 0;
	delay_us(5);
	for(int i=0;i<len;++i)
	{
		ps2_resp_data[i]=sendCommmandAndGetResp(byte[i]);
	}
	CS = 1;
	return ps2_resp_data[1];

}

void reset_resp_data(void)
{
	for(int i=0;i<21;++i)
		ps2_resp_data[i]=0;
}

void ps2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  //DAT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  //CS CMD CLK
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_3|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  CMD = 1;
  CLK = 1;
	CS =1;

}



