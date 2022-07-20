#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"
#include "ps2lib.h"
#include "ps2.h"
//#define sendIMUByInterrupt	

u8 cmd[9] = {0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


int main(void)
{		
	
	board_configInit();
	param_init();
	PS2_Init();
//		for(int i=0;i<5;++i)
//	{
//		CMD = 1;
//		CLK = 1;
//		CS = 0;
//	}
	while(1)
	{	
//		
//		security_mode();

//#ifndef sendIMUByInterrupt	
////		if(imu_ready_flag==(short)0xffff)
////		{
////			uart_lock = 1;
////			sendFrame_IMU_Raw_dpkg(&imu_raw_msg);
////			imu_ready_flag=0x000f;
////			if(uart_lock ==2)
////				sendFrame_Encoder_dpkg(&robot_msgs);
////			uart_lock = 0;
////		}
//#endif

//		voltage_check();
//		oled_showContent();
		
		
//		sendCommandLists(cmd,9);

//		oled_digit(0,0,ps2_resp_data[0],3,12);
//		oled_digit(0,12,ps2_resp_data[1],3,12);
//		oled_digit(0,24,ps2_resp_data[2],3,12);
//		oled_digit(0,36,ps2_resp_data[3],3,12);
//		oled_digit(0,48,ps2_resp_data[4],3,12);
//		oled_digit(30,0,ps2_resp_data[5],3,12);
//		oled_digit(30,12,ps2_resp_data[6],3,12);
//		oled_digit(30,24,ps2_resp_data[7],3,12);
//		oled_digit(30,36,ps2_resp_data[8],3,12);

PS2_ReadData();
		oled_digit(0,0,Data[0],3,12);
		oled_digit(0,12,Data[1],3,12);
		oled_digit(0,24,Data[2],3,12);
		oled_digit(0,36,Data[3],3,12);
		oled_digit(0,48,Data[4],3,12);
		oled_digit(30,0,Data[5],3,12);
		oled_digit(30,12,Data[6],3,12);
		oled_digit(30,24,Data[7],3,12);
		oled_digit(30,36,Data[8],3,12);
		oled_update();
		PS2_ClearData();
	}
}











