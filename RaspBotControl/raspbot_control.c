#include "raspbot_control.h"
#include "raspbot_config.h"
#include "raspbot_comm.h"
#include "math.h"
#include "stdlib.h"
#include "ps2lib.h"

/* global params */
Robot_msgs robot_msgs;
Motor_msgs motor_msgs;
IMU_Raw_msg imu_raw_msg;
PID pid;
Battery batteryState = BATTERY_FULL;


/* time flag */
u32 flag_500ms = 0;

void time_flag(void)
{
	// time flag
	++flag_500ms;
	if (flag_500ms > 50)
	{
		STATE_LED = !STATE_LED;
		flag_500ms = 0;
	}
}

void voltage_check(void)
{
	u32 value = 0;
	for (int i = 1; i <= COV_COUNT; ++i)
	{
		value += getAnalogValue();
	}
	float voltage = ((value / COV_COUNT) / 4096.0) * ANALOG_VOL * VOL_SCALE;
	//更新电压
	if (voltage - robot_msgs.voltage > BAT_RESOLUTION || robot_msgs.voltage - voltage > BAT_RESOLUTION)
		robot_msgs.voltage = voltage;

	if (robot_msgs.voltage <= LOW_VOLTAGE)
		batteryState = BATTERY_ALARM;
	else if (robot_msgs.voltage > 11.85)
		batteryState = BATTERY_FULL;
	else if (robot_msgs.voltage > 11.1 && robot_msgs.voltage <= 11.85)
		batteryState = BATTERY_MEDIUM;
	else if (robot_msgs.voltage > 10.35 && robot_msgs.voltage <= 11.1)
		batteryState = BATTERY_LESS;
	else if (robot_msgs.voltage > 9.6 && robot_msgs.voltage <= 10.35)
		batteryState = BATTERY_LOW;
}

void lowVoltageAlarm(void)
{
	if (flag_500ms == 50)
	{
		oled_picture(40, 20, 48, 24, LOW_BATTERY);
		flag_500ms = 0;
	}
	else if (flag_500ms == 1)
		OLED_Clear();
}

void oled_showContent(void)
{
	//显示电压
//	oled_float(74, 0, robot_msgs.voltage, 1, 3, 12);
//	oled_char(98, 0, 'V', 12, 1);
	switch (batteryState)
	{
	case BATTERY_FULL:
		oled_picture(104, 0, 24, 12, BATTERY_4);
		break;
	case BATTERY_MEDIUM:
		oled_picture(104, 0, 24, 12, BATTERY_3);
		break;
	case BATTERY_LESS:
		oled_picture(104, 0, 24, 12, BATTERY_2);
		break;
	case BATTERY_LOW:
		oled_picture(104, 0, 24, 12, BATTERY_1);
		break;
	case BATTERY_ALARM: /*lowVoltageAlarm();*/
		break;
	}

	//显示受控速度

	if (motor_msgs.velocity >= 0.0)
		oled_char(98, 36, '+', 12, 1);
	else
		oled_char(98, 36, '-', 12, 1);
	if (motor_msgs.angular >= 0.0)
		oled_char(98, 52, '+', 12, 1);
	else
		oled_char(98, 52, '-', 12, 1);
	oled_float(104, 36, motor_msgs.velocity, 1, 2, 12);
	oled_float(104, 52, motor_msgs.angular, 1, 2, 12);


	//显示编码器速度
	if (robot_msgs.l_encoder_pulse >= 0.0)
		oled_char(0, 40, '+', 12, 1);
	else
		oled_char(0, 40, '-', 12, 1);
	if (robot_msgs.r_encoder_pulse >= 0.0)
		oled_char(0, 52, '+', 12, 1);
	else
		oled_char(0, 52, '-', 12, 1);
	oled_digit(7, 40, robot_msgs.l_encoder_pulse, 4, 12);
	oled_digit(7, 52, robot_msgs.r_encoder_pulse, 4, 12);
	
	//PID
//	oled_char(0,0,'P',12,0);oled_float(8,0,pid.Kp, 1, 4,12);
//	oled_char(0,13,'I',12,0);oled_float(8,13, pid.Ki, 1, 2,12);
//	oled_char(0,26,'D',12,0);oled_float(8,26, pid.Kd, 1, 3,12);
	

	oled_update();
}

/**
	* @brief
	* @attention Can't put in interrupt,because may be reading gamepad status while IT triggering
	*/
void ps2_control(void)
{
	 //speed controled by gamepad
  //the speed cmd from computer will be covered
	if(IS_JOYSTICK_MODE(ps2_mode))
	{
		int speed_analog_value  = JOYSTICK_INIT_VALUE - getJoyAnalogValue(PSS_LY);
		speed_analog_value =abs(speed_analog_value)<2?0:speed_analog_value;
		motor_msgs.velocity = speed_analog_value>=0?speed_analog_value*joy_forward_scale:speed_analog_value*joy_backward_scale;
		//nonlinearity
		if(abs(speed_analog_value)<50)
			motor_msgs.velocity*=1.2;
		
		int angular_analog_value = JOYSTICK_INIT_VALUE - getJoyAnalogValue(PSS_RX);
		angular_analog_value =abs(angular_analog_value)<2?0:angular_analog_value;
		motor_msgs.angular = angular_analog_value*joy_steering_scale;
//		//nonlinearity
//		if(abs(angular_analog_value)<50)
//			motor_msgs.angular*=1.2;
//		
		// accelerate
		if(getButtonStatus(PSB_R1))
		{
			motor_msgs.velocity *=2.0;
			motor_msgs.angular*=2.5;
		}
		//Braking
		if(getButtonStatus(PSB_L1))
		{
			motor_msgs.velocity = 0.0;
			motor_msgs.angular = 0.0;
		}
	}
	else if(IS_DISCONNECTED(ps2_mode))
	{
		motor_msgs.velocity=0.0;
		motor_msgs.angular = 0.0;
	}
	else
	{
		//ps2 digital mode is used to  interaction  with mcu as keyboard

		
		
	}
}
void speed_control()
{

   //limit speed
  if(motor_msgs.velocity > MAX_SPEED) motor_msgs.velocity=MAX_SPEED;
  else if(motor_msgs.velocity < -MAX_SPEED*0.6) motor_msgs.velocity=-MAX_SPEED*0.6;

  if(motor_msgs.angular > MAX_STEERING) motor_msgs.angular=MAX_STEERING;
  else if(motor_msgs.angular < -MAX_STEERING) motor_msgs.angular=-MAX_STEERING;

	// speed resolution
	float L_velocity = motor_msgs.velocity - motor_msgs.angular * wheelTrack / 2;
	float R_velocity = motor_msgs.velocity + motor_msgs.angular * wheelTrack / 2;
	int16_t L_encoderSet = L_velocity * intervalTimer * PPR / (2 * wheelRadius * M_PI); 
	int16_t R_encoderSet = R_velocity * intervalTimer * PPR / (2 * wheelRadius * M_PI); 



    //PID control
	pid.error[0] = L_encoderSet - robot_msgs.l_encoder_pulse;
	pid.error[1] = R_encoderSet - robot_msgs.r_encoder_pulse;
	
	pid.accumu_error[0]+=pid.error[0];
	pid.accumu_error[1]+=pid.error[1];
	
	pid.out_pwm[0] = pid.Kp*pid.error[0]+pid.Ki*pid.accumu_error[0]+pid.Kd*(pid.error[0]-pid.last_error[0]);
	pid.out_pwm[1] = pid.Kp*pid.error[1]+pid.Ki*pid.accumu_error[1]+pid.Kd*(pid.error[1]-pid.last_error[1]);



	pid.last_error[0]=pid.error[0];
	pid.last_error[1]=pid.error[1];
 
	//OUT limit
	if(pid.out_pwm[0]>pid.limMax) pid.out_pwm[0]=pid.limMax;
	else if(pid.out_pwm[0]<pid.limMin && pid.out_pwm[0]>0) pid.out_pwm[0]=pid.limMin;
	if(pid.out_pwm[1]>pid.limMax) pid.out_pwm[1]=pid.limMax;
	else if(pid.out_pwm[1]<pid.limMin && pid.out_pwm[1]>0) pid.out_pwm[1]=pid.limMin;

	if(pid.out_pwm[0]<-pid.limMax) pid.out_pwm[0]=-pid.limMax;
	else if(pid.out_pwm[0]>-pid.limMin && pid.out_pwm[0]<0) pid.out_pwm[0]=-pid.limMin;
	if(pid.out_pwm[1]<-pid.limMax) pid.out_pwm[1]=-pid.limMax;
	else if(pid.out_pwm[1]>-pid.limMin && pid.out_pwm[1]<0) pid.out_pwm[1]=-pid.limMin;
	


	motor_pwm(pid.out_pwm[0],pid.out_pwm[1]);
	

}

/**
 * @brief  定时器1中断入口，10ms中断
 *         程序控制逻辑，控制电机等其他中枢
 */
void TIM1_UP_IRQHandler(void)
{
	//清除标志位
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	
	//时间标志
	time_flag();

	//读取编码器
	robot_msgs.l_encoder_pulse = -Read_Encoder(2);
	robot_msgs.r_encoder_pulse = +Read_Encoder(4);


	//速度解算与PID控制
	speed_control();

	//串口发送数据4
#ifndef sendIMUByInterrupt
	if (!uart_lock)
		sendFrame_Encoder_dpkg(&robot_msgs);
	else
		uart_lock = 2;
	
		sendFrame_Encoder_dpkg(&robot_msgs);
#else
	sendFrame_Multi_dpkg();
#endif
	
	

}



void param_init(void)
{
	robot_msgs.voltage = 11.1;
	robot_msgs.l_encoder_pulse = 0;
	robot_msgs.r_encoder_pulse = 0;
	robot_msgs.acc[0]=0;
	robot_msgs.acc[1]=0;
	robot_msgs.acc[2]=0;
	
	robot_msgs.gyr[0]=0;
	robot_msgs.gyr[1]=0;
	robot_msgs.gyr[2]=0;

#ifdef imu_mag
	robot_msgs.mag[0]=0;
	robot_msgs.mag[1]=0;
	robot_msgs.mag[2]=0;
#endif

	robot_msgs.elu[0]=0;
	robot_msgs.elu[1]=0;
	robot_msgs.elu[2]=0;


	pid.Kp = 26.0;
	pid.Ki = 2.5;
	pid.Kd = 5.0;
	
	pid.limMin = 80;
	pid.limMax = 999;

	pid.error[0] = 0.0;
	pid.error[1] = 0.0;

	pid.last_error[0] = 0.0;
	pid.last_error[1] = 0.0;

	pid.accumu_error[0]=0.0;
	pid.accumu_error[1]=0.0;

	pid.out_pwm[0]=0;
	pid.out_pwm[1]=0;

	motor_msgs.angular = 0.0;
	motor_msgs.velocity = 0.0;

	imu_raw_msg.accRaw[0]=0;
	imu_raw_msg.accRaw[1]=0;
	imu_raw_msg.accRaw[2]=0;

	imu_raw_msg.gyrRaw[0]=0;
	imu_raw_msg.gyrRaw[1]=0;
	imu_raw_msg.gyrRaw[2]=0;

#ifdef imu_mag
	imu_raw_msg.magRaw[0]=0;
	imu_raw_msg.magRaw[1]=0;
	imu_raw_msg.magRaw[2]=0;
#endif

	imu_raw_msg.eluRaw[0]=0;
	imu_raw_msg.eluRaw[1]=0;
	imu_raw_msg.eluRaw[2]=0;

	
}
