#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"
#include "ps2lib.h"

//#define sendIMUByInterrupt	
int a  = 0;
int main(void)
{		
	
	board_configInit();
	param_init();
	
	while(1)
	{	
		
		security_mode();
		voltage_check();
		
		switch(ps2_mode)
		{
			case DIGITAL_MODE: oled_char(0,0,'D',12,1); break;
			case ANALOG_MODE:  oled_char(0,0,'A',12,1); break;
			case CONFIG_MODE:  oled_char(0,0,'C',12,1); break;
			case DISCONNECTED: oled_char(0,0,'N',12,1); break;
		}
		if(buttonPressed(PSB_PAD_UP))
			++a;
		oled_digit(0,12,a,3,12);
		oled_showContent();
	}
}











