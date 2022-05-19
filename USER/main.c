#include "raspbot_config.h"
#include "raspbot_control.h"

int main(void)
{		
	board_configInit();

	while(1)
	{	
		test();
//		voltage_check();
//		oled_showContent();
		delay_ms(1);
	}
}











