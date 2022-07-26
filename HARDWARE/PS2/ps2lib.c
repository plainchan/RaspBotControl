#include "stm32f10x_gpio.h"
#include "ps2lib.h"
#include "stdint.h"
#include "delay.h"
#include "oled.h"

static const uint8_t enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const uint8_t exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
//static const uint8_t type_read[] = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
//static const uint8_t set_mode[] = {0x01, 0x44, 0x00, /* enabled */ 0x01, /* locked */ 0x03, 0x00, 0x00, 0x00, 0x00};
static const uint8_t enable_rumble[] = {0x01, 0x4D, 0x00, /* motor 1 on */ 0x00, /* motor 2 on*/ 0x01, 0xff, 0xff, 0xff, 0xff};
static const uint8_t set_pressures[] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};

static const uint8_t query_command[21]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static uint8_t ps2_resp_data[21];
static uint16_t buttons_status = 0xFF,last_buttons_status = 0xFF;   //
static uint8_t en_Rumble = 0;
static uint8_t en_Pressures = 0;
static uint8_t resp_len = 21;

PS2_MODE ps2_mode;
	
uint8_t sendCommmandAndGetResp(const uint8_t byte)
{
	uint8_t res = 0;

	for(int i=0;i<8;++i)
	{
		//单片机向手柄发送命令
		if(IS_BIT_SET(byte,i))CMD=1;
		else CMD = 0;             

		// 在时钟下降沿时，完成数据（1bit）的发送与接收
		CLK = 0;

		delay_us(CLK_DELAY);

		if(DAT) BIT_SET(res,i);
		
		CLK = 1;
		delay_us(CLK_DELAY);
	}
	CMD = 1;
#ifdef SEND_BYTE_DELAY
	delay_us(SEND_BYTE_DELAY);
#endif
	return res;
}

void reset_resp_data(void)
{
	for(int i=0;i<21;++i)
		ps2_resp_data[i]=0;
}

uint8_t  sendCommandLists(const uint8_t *byte,uint8_t len)
{
	reset_resp_data();
	CS = 0;
	delay_us(CLK_DELAY);
	for(int i=0;i<len;++i)
	{
		ps2_resp_data[i]=sendCommmandAndGetResp(byte[i]);
	}
	CS = 1;
	return ps2_resp_data[1];

}


void read_gamepad(void)
{
    
    reset_resp_data();
	
	CS = 0;
	delay_us(CLK_DELAY);
    for(int i=0;i<21;++i)
    {
        if(i>resp_len) break;

        ps2_resp_data[i] =  sendCommmandAndGetResp(query_command[i]);

        //check mode
        if(i==1)
        {
            switch (ps2_resp_data[1])
            {
            case 0x41: ps2_mode = DIGITAL_MODE; resp_len = 5; break;
            case 0x73: ps2_mode = ANALOG_MODE; resp_len = 9; break;
            case 0x79: ps2_mode = ANALOG_MODE; resp_len = 21; break;
            case 0xF3: ps2_mode = CONFIG_MODE; resp_len = 9; break;
            default: ps2_mode = DISCONNECTED; resp_len = 2; break;
            }
        }
    }
	CS = 1;
    last_buttons_status = buttons_status;
    buttons_status = (uint16_t)ps2_resp_data[4]<<8 | ps2_resp_data[3];
    
}
/**
 * @brief Get the Button Status,keeping triggering while pressed
 * 
 * @param button 
 * @return uint8_t 
 */
uint8_t getButtonStatus(const uint16_t button)
{
    return (~buttons_status&button)>0;
}

/**
 * @brief Get the Button Status comparing with the last time
 * 
 * @return uint8_t 
 */
uint8_t isButtonStatusChanged(const uint16_t button)
{
    return ((buttons_status^last_buttons_status)&button)>0;
}
/**
 * @brief Get the Button Status comparing with the last time ,trigger once while pressed
 * 
 * @param button 
 * @return uint8_t 
 */
uint8_t buttonPressed(const uint16_t button)
{               
    return isButtonStatusChanged(button) & getButtonStatus(button);
}
/**
 * @brief Get the Button Status,trigger once while released
 * 
 * @param button 
 * @return uint8_t 
 */
uint8_t buttonReleased(const uint16_t button)
{
    return isButtonStatusChanged(button) & ((~last_buttons_status & button) > 0);
    //        button change             &&        at last time the  button is pressed
}

uint8_t getJoyAnalogValue(const uint8_t button)
{
    if(ps2_mode == ANALOG_MODE && button <21)
        return ps2_resp_data[button];
    else 
        return 0;
}

void reconfig_gamepad(void)
{
    sendCommandLists(enter_config,sizeof(enter_config));
    if(en_Rumble)
        sendCommandLists(enable_rumble,sizeof(enable_rumble));
    if(en_Pressures)
        sendCommandLists(set_pressures,sizeof(set_pressures));
    sendCommandLists(exit_config,sizeof(exit_config));
}

void config_gamepad(void)
{

}



void ps2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  //DAT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
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
	delay_us(CLK_DELAY);
}



