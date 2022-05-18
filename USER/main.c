#include "raspbot_config.h"
#include "raspbot_control.h"

int num;
int a=0;
Frame_Robot_msg msg;

int main(void)
{		
	board_configInit();
	msg.header[0]=Header1;
	msg.header[1]=Header2;
	msg.len = 55;
	msg.crc = 222;
	msg.robot_msgs.data_tag=robot_tag;
	msg.robot_msgs.voltage = 115;
	msg.robot_msgs.l_encoder_pulse = 0;
	msg.robot_msgs.r_encoder_pulse = 0;
	msg.robot_msgs.acc[0]=1.0;
	msg.robot_msgs.acc[1]=1.0;
	msg.robot_msgs.acc[2]=1.0;
	msg.robot_msgs.gyr[0]=2.0;
	msg.robot_msgs.gyr[1]=2.0;
	msg.robot_msgs.gyr[2]=2.0;
	msg.robot_msgs.mag[0]=3.0;
	msg.robot_msgs.mag[1]=3.0;
	msg.robot_msgs.mag[2]=3.0;
	msg.robot_msgs.elu[0]=4.0;
	msg.robot_msgs.elu[1]=4.0;
	msg.robot_msgs.elu[2]=4.0;
	while(1)
	{	
//		oled_num(0,0,++a,3,12);
//		voltage_check();
//		oled_showContent();
		sendFrameData(&msg);
//		delay_ms(20);
		
		msg.robot_msgs.acc[0]+=1.0;
		msg.robot_msgs.acc[1]+=1.0;
		msg.robot_msgs.acc[2]+=1.0;
		msg.robot_msgs.gyr[0]+=1.0;
		msg.robot_msgs.gyr[1]+=1.0;
		msg.robot_msgs.gyr[2]+=1.0;
		msg.robot_msgs.mag[0]+=1.0;
		msg.robot_msgs.mag[1]+=1.0;
		msg.robot_msgs.mag[2]+=1.0;
		msg.robot_msgs.elu[0]+=1.0;
		msg.robot_msgs.elu[1]+=1.0;
		msg.robot_msgs.elu[2]+=1.0;
	}
}











