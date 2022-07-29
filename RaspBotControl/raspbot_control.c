#include "raspbot_control.h"
#include "raspbot_config.h"
#include "raspbot_comm.h"
#include "math.h"
#include "stdlib.h"
#include "ps2lib.h"
#include "stdbool.h"
/* global params */
Robot_msgs robot_msgs;
Motor_msgs motor_msgs;
IMU_Raw_msg imu_raw_msg;
PID pid;
Battery batteryState = BATTERY_FULL;

u32 tick_clock = 0;


/**
 * @brief 间隔设置时间，开关切换一次
 * 		  分辨率定时器中断间隔
 * 
 * @param ms 间隔设置时间
 * @return u8  true or false
 */
u8 duration_trigger_switch(u32 ms)
{
		static u8 flag = 1;
		static u32 clock = 0;
		static u8 trigger = 1;
		if(flag)
		{
			clock = tick_clock;
			flag = 0;		
		}
		else if(tick_clock-clock>=ms)
		{
			flag = 1;
			trigger = !trigger;
		}
		return trigger;
}
/**
 * @brief  间隔设置时间，触发一次，常开开关
 * 
 * @param ms 
 * @return u8 
 */
u8 duration_trigger_once(u32 ms)
{
		static u8 start = 1;
		static u32 clock = 0;
		if(start)
		{
			clock = tick_clock;
			start = 0;		
		}
		else if(tick_clock-clock>=ms)
		{
			start =1; 
			return 1;	
		}
		return 0;
}

/**
 * @brief 长按按键检测，触发一次
 * 
 * @param button 按键
 * @param ms  长按触发时间间隔
 * @return u8 
 */
u8 longPressButton_triggerOnce(u16 button,u32 ms)
{
		if(ms<1000)
			return 0;
		
		static u32 clock = 0;
		static u8  trigger = 0;
		
		if(buttonPressed(button))
		{
			clock = tick_clock;		
		}
		
		if(trigger && !buttonReleased(button))
			return !trigger;
		else
			trigger =0;
		
		if(!getButtonStatus(button))
			clock = tick_clock;
		else if(tick_clock-clock>ms)
		{
			clock = tick_clock;
			trigger =1;
		}
		return trigger;
}

/**
 * @brief 长按按键检测，保持触发状态
 * 
 * @param button 按键
 * @param ms 长按触发时间间隔
 * @return u8 
 */
u8 longPressButton_trigger(u16 button,u32 ms)
{
		if(ms<1000)
			return 0;
		
		static u32 clock = 0;

		
		if(buttonPressed(button))
		{
			clock = tick_clock;		
		}
		
		if(!getButtonStatus(button))
			clock = tick_clock;
		else if(!buttonReleased(button)&&tick_clock-clock>ms)
			return 1;
		
		return 0;
}
/**
 * @brief 
 * 
 * @param button 
 * @param ms 
 * @return u8 
 */
u8 pressButton_triggerPeriodic(u16 button,u32 ms)
{
		if(ms<=100)
			ms=100;
		
		static u32 clock = 0;

		
		if(buttonPressed(button))
		{
			clock = tick_clock;		
		}
		
		if(!getButtonStatus(button))
			clock = tick_clock;
		else if(tick_clock-clock>ms)
		{
			clock = tick_clock;
			return 1;
		}
		
		return 0;
}


/**
 * @brief 运行状态指示灯
 * 
 */
void state_led(void)
{
	if(duration_trigger_once(STATE_LED_NORMAL_DURATION))
		STATE_LED =!STATE_LED;
		
}

void voltage_check(void)
{
	static u32 value = 0;
	static u16 check_count = 0;
	if(++check_count<= COV_COUNT)
	{
		value += getAnalogValue();
		return;
	}
	
	float voltage = VOL_SCALE*ANALOG_VOL*value / (COV_COUNT*4096);
	//更新电压
	if (voltage - robot_msgs.voltage > BAT_RESOLUTION || robot_msgs.voltage - voltage > BAT_RESOLUTION)
		robot_msgs.voltage = voltage;

	if (robot_msgs.voltage <= LOW_VOLTAGE)
	{
		batteryState = BATTERY_ALARM;
//		safe_mode = true;
	}
	else if (robot_msgs.voltage > 11.85)
		batteryState = BATTERY_FULL;
	else if (robot_msgs.voltage > 11.1 && robot_msgs.voltage <= 11.85)
		batteryState = BATTERY_MEDIUM;
	else if (robot_msgs.voltage > 10.35 && robot_msgs.voltage <= 11.1)
		batteryState = BATTERY_LESS;
	else if (robot_msgs.voltage > 9.6 && robot_msgs.voltage <= 10.35)
		batteryState = BATTERY_LOW;
	
	check_count = 0;
	value = 0;
}

/**
 * @brief  安全模式，长按按键上锁/解锁
 */
volatile char safe_mode = 0;


void oled_showContent(void)
{
	//安全模式，长按按键上锁/解锁
	if(safe_mode)
		oled_picture(88,0,12,12,safe_lock);
	else
		oled_str(88,0,"  ",12);
	
	
    // // 显示电压
    // oled_float(74, 0, robot_msgs.voltage, 1, 3, 12);
    // oled_char(98, 0, 'V', 12, 1);
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
	case BATTERY_ALARM:
			if(duration_trigger_switch(500))
				oled_picture(104, 0, 24, 12, BATTERY_0);
			else
				oled_str(104,0,"    ",12);	
			break;
		
	}

	//显示受控速度
	oled_float(104, 36, motor_msgs.velocity, 1,12);
	oled_float(104, 52, motor_msgs.angular, 1,12);


	//显示编码器速度
	oled_digit(0,40, robot_msgs.l_encoder_pulse,12);
	oled_digit(0,52, robot_msgs.r_encoder_pulse,12);
	motor_msgs.velocity = 0.0;
	motor_msgs.angular = 0.0;
	motor_msgs.attribution = ATTRIBUTION_NONE_SPEED;
	
	// PID
	// oled_char(0,0,'P',12,0);oled_float(8,0,pid.Kp, 1, 4,12);
	// oled_char(0,13,'I',12,0);oled_float(8,13, pid.Ki, 1, 2,12);
	// oled_char(0,26,'D',12,0);oled_float(8,26, pid.Kd, 1, 3,12);
	
    //显示ps2
    switch(ps2_mode)
    {
        case DIGITAL_MODE: oled_picture(0,0,24,12,buttonPad); break;
        case ANALOG_MODE:  oled_picture(0, 0, 24, 12, gamepad); break;
        case CONFIG_MODE:  oled_picture(0, 0, 24, 12, buttonConfig); break;
        case DISCONNECTED: oled_str(0,0,"    ",12); break;
    }

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
		// Accelerate
		if(getButtonStatus(PSB_R1))
		{
			motor_msgs.velocity *=2.0;
			motor_msgs.angular*=2.5;
		}
		// Braking
		if(getButtonStatus(PSB_L1))
		{
			motor_msgs.velocity = 0.0;
			motor_msgs.angular = 0.0;
		}
		
		motor_msgs.attribution = ATTRIBUTION_JOYSTICK_SPEED;
	}
	else if(IS_DISCONNECTED(ps2_mode))
	{
		if(motor_msgs.attribution != ATTRIBUTION_COMPUTER_SPEED)
		{
			motor_msgs.velocity=0.0;
			motor_msgs.angular = 0.0;
			motor_msgs.attribution = ATTRIBUTION_DISCONNECT_SPEED;
		}
	}
	else
	{
		//ps2 digital mode is used to  interaction  with mcu as keyboard
		
		//safe lock

		if(longPressButton_triggerOnce(PSB_L2,2000))
		{
			safe_mode=!safe_mode;
		}
	}
}
void speed_control()
{
	if(safe_mode)
	{
		motor_pwm(0,0);
		MOTOR_ENABLE(DISABLE);
	}
	else
	{
		 //limit speed
		MOTOR_ENABLE(ENABLE);
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
	tick_clock+=10;
	
	state_led();

	//读取编码器
	robot_msgs.l_encoder_pulse = -Read_Encoder(2);
	robot_msgs.r_encoder_pulse = +Read_Encoder(4);

	read_gamepad();    //0.16ms(2 bytes),0.4ms(5 bytes),0.72ms(9 bytes),1.68ms(21 bytes)
	ps2_control();

	//速度解算与PID控制
	speed_control();
	
	//电压检测
	voltage_check();
	
	//send data
	sendFrame_Encoder_dpkg(&robot_msgs);  //0.88ms
}



void param_init(void)
{
	robot_msgs.voltage = 12.6;
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
