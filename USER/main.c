#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"
<<<<<<< HEAD

=======
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee

int main(void)
{		
	board_configInit();
	while(1)
	{	
<<<<<<< HEAD
		++robot_msgs.l_encoder_pulse;++robot_msgs.r_encoder_pulse;
		++robot_msgs.acc[0];++robot_msgs.acc[1];++robot_msgs.acc[2];
		++robot_msgs.gyr[0];++robot_msgs.gyr[1];++robot_msgs.gyr[2];
		++robot_msgs.mag[0];++robot_msgs.mag[1];++robot_msgs.mag[2];
		++robot_msgs.elu[0];++robot_msgs.elu[1];++robot_msgs.elu[2];
		
//		sendFrame_Robot_dpkg(&robot_msgs);
//		sendFrame_IMU_dpkg(&robot_msgs);
//		sendFrame_IMU_Sensor_dpkg(&robot_msgs);
//		sendFrame_Encoder_dpkg(&robot_msgs);
		oled_char(0,0,'!',12,1);
		oled_char(6,0,'i',12,1);
		voltage_check();
		
		oled_showContent();
		

=======
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
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee
	}
}











