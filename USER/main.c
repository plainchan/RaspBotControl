#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"

int main(void)
{		
	board_configInit();
	while(1)
	{	
		if(imu_ready_flag==(short)0xfff0)
		{
			sendFrame_IMU_Raw_dpkg(&imu_raw_msg);
			imu_ready_flag=0;
		}
		voltage_check();
		oled_showContent();
	}
}











