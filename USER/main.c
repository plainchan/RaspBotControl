#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"

int main(void)
{		
	board_configInit();
	while(1)
	{	
//		robot_msgs.voltage+=0.1;
//		if((uint8_t)(robot_msgs.voltage*10)>255)
//			robot_msgs.voltage=0.0;
		++robot_msgs.acc[0];++robot_msgs.acc[1];++robot_msgs.acc[2];
		++robot_msgs.gyr[0];++robot_msgs.gyr[1];++robot_msgs.gyr[2];
		++robot_msgs.elu[0];++robot_msgs.elu[1];++robot_msgs.elu[2];
		++robot_msgs.mag[0];++robot_msgs.mag[1];++robot_msgs.mag[2];
		robot_msgs.r_encoder_pulse=robot_msgs.l_encoder_pulse++;
		sendFrame_Robot_dpkg(&robot_msgs);
		
		oled_float(0,0,112.568,3,6,12);
		oled_float(0,0,1.1,1,6,12);
		voltage_check();
		oled_showContent();
	}
}











