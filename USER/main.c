#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"
#include "ps2lib.h"
#include "ps2.h"
//#define sendIMUByInterrupt	

int main(void)
{		
	
	board_configInit();
	param_init();
	
	while(1)
	{	
		
		security_mode();

#ifndef sendIMUByInterrupt	
//		if(imu_ready_flag==(short)0xffff)
//		{
//			uart_lock = 1;
//			sendFrame_IMU_Raw_dpkg(&imu_raw_msg);
//			imu_ready_flag=0x000f;
//			if(uart_lock ==2)
//				sendFrame_Encoder_dpkg(&robot_msgs);
//			uart_lock = 0;
//		}
#endif

		voltage_check();
        read_gamepad();
		oled_showContent();
	}
}











