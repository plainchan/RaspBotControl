#include "raspbot_config.h"
#include "raspbot_control.h"

int main(void)
{		
	board_configInit();

	while(1)
	{	
		
		sendFrame_Encoder_dpkg(&frame_encode_dpkg,&robot_msgs,0);
//		voltage_check();
//		oled_showContent();
//		delay_ms(1);
	}
}











