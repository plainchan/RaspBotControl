#include "raspbot_config.h"
#include "raspbot_control.h"
#include "raspbot_comm.h"
#include "ps2lib.h"


int main(void)
{		
	
	board_configInit();
	param_init();
	while(1)
	{	
		oled_showContent();
	}
}











