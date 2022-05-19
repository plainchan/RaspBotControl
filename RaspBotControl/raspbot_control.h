#ifndef __RASPBOT_CONTROL_H__
#define __RASPBOT_CONTROL_H__


#include "raspbot_comm.h"

#define ANALOG_VOL    3.30         //ģ���ѹ�ο�ֵ
#define COV_COUNT     5.0          //��ѹѭ��������
#define VOL_SCALE     4.333        //��ѹ����     10K/3K+1
#define LOW_VOLTAGE   9.6          //��͵�ѹ
#define BAT_RESOLUTION 0.05


typedef enum
{
	BATTERY_FULL,
	BATTERY_MEDIUM,
	BATTERY_LESS,
	BATTERY_LOW,
	BATTERY_ALARM
}Battery;

extern volatile float power_voltage;
void voltage_check(void);
void oled_showContent(void);
void lowVoltageAlarm(void);
void test(void);

extern Stream_msgs               stream_msgs;
extern Frame_Robot_dpkg          robot_dpkg;
extern Frame_IMU_dpkg            imu_dpkg;
extern Frame_IMU_9Axis_dpkg 		 imu_9Axis_dpkg;
extern Frame_IMU_6Axis_dpkg 		 imu_6Axis_dpkg;
extern Frame_Encoder_dpkg 			 encode_dpkg;
extern Frame_Voltage_dpkg 			 voltage_dpkg;









#endif
