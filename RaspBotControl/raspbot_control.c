#include "raspbot_control.h"
#include "raspbot_config.h"
#include "raspbot_comm.h"

/* global params */
Robot_msgs     robot_msgs={0};
Motor_msgs     motor_msgs={0.0,0.0,0,0};
IMU_Raw_msg    imu_raw_msg={0};
PID            pid={0};
Battery batteryState=BATTERY_FULL;

/* time flag */
u32 flag_500ms=0;

void time_flag(void)
{
	//time flag
	++flag_500ms;
	if(flag_500ms>50)
	{
		STATE_LED=!STATE_LED;
		flag_500ms=0;
	}
}


void voltage_check(void)
{
	u32 value=0;
	for(int i=1;i<=COV_COUNT;++i)
	{
		value+=getAnalogValue();
	}
	float voltage = ((value/COV_COUNT)/4096.0)*ANALOG_VOL*VOL_SCALE;
	//更新电压
	if(voltage-robot_msgs.voltage>BAT_RESOLUTION || robot_msgs.voltage-voltage>BAT_RESOLUTION)
		robot_msgs.voltage = voltage;
	
	if(robot_msgs.voltage<=LOW_VOLTAGE)
		batteryState=BATTERY_ALARM;	
	else if(robot_msgs.voltage>11.85)
		batteryState=BATTERY_FULL;
	else if(robot_msgs.voltage>11.1 && robot_msgs.voltage<=11.85)
		batteryState=BATTERY_MEDIUM;
	else if(robot_msgs.voltage>10.35 && robot_msgs.voltage<=11.1)
		batteryState=BATTERY_LESS;
	else if(robot_msgs.voltage>9.6 && robot_msgs.voltage<=10.35)
		batteryState=BATTERY_LOW;
}

void lowVoltageAlarm(void)
{
	if(flag_500ms==50)
	{
		oled_picture(40,20,48,24,LOW_BATTERY);
		flag_500ms=0;
	}
	else if(flag_500ms==1)
		OLED_Clear();
}


void oled_showContent(void)
{
	//显示电压
	oled_float(74,0,robot_msgs.voltage,1,3,12);
	oled_char(98,0,'V',12,1);
	switch(batteryState)
	{
		case BATTERY_FULL:oled_picture(104,0,24,12,BATTERY_4); break;
		case BATTERY_MEDIUM:oled_picture(104,0,24,12,BATTERY_3); break;
		case BATTERY_LESS:oled_picture(104,0,24,12,BATTERY_2); break;
		case BATTERY_LOW:oled_picture(104,0,24,12,BATTERY_1); break;
		case BATTERY_ALARM:/*lowVoltageAlarm();*/ break;
	}
	
   //显示受控速度
 	if(receiveFlag==1)
 	{
 		if(motor_msgs.velocity>=0.0) oled_char(98,36,'+',12,1); else oled_char(98,36,'-',12,1);
 		if(motor_msgs.yaw>=0.0) oled_char(98,52,'+',12,1); else oled_char(98,52,'-',12,1);
 		oled_float(104,36,motor_msgs.velocity,1,2,12);
 		oled_float(104,52,motor_msgs.yaw,1,2,12);
 	}
	
	
	//显示编码器速度
	if(robot_msgs.l_encoder_pulse>=0.0) oled_char(0,36,'+',12,1); else oled_char(0,36,'-',12,1);
 	if(robot_msgs.r_encoder_pulse>=0.0) oled_char(0,52,'+',12,1); else oled_char(0,52,'-',12,1);
	oled_digit(7,36,robot_msgs.l_encoder_pulse,4,12);
	oled_digit(7,52,robot_msgs.r_encoder_pulse,4,12);
	
	oled_update();
}



void calculate_plus()
{
	
}

void speed_control()
{
	
}







/**
  * @brief  定时器1中断入口，10ms中断
	*         程序控制逻辑，控制电机等其他中枢
  */
void TIM1_UP_IRQHandler(void)
{
	//清除标志位
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	
	
	//读取编码器
	robot_msgs.l_encoder_pulse = Read_Encoder(2);
	robot_msgs.r_encoder_pulse = -Read_Encoder(4);
	
	//发送 编码器
#ifndef sendIMUByInterrupt
	if(!uart_lock)
		sendFrame_Encoder_dpkg(&robot_msgs);
	else
		uart_lock = 2;
#else
	sendFrame_Multi_dpkg();
#endif
	
	//速度解算与PID控制
	
	
	//时间标志
	time_flag();
	

}







