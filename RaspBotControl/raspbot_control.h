#ifndef __RASPBOT_CONTROL_H__
#define __RASPBOT_CONTROL_H__

#include "stdint.h"

//#define imu_mag

#define  G  9.780

#define ANALOG_VOL    3.30         //模拟电压参考值
#define COV_COUNT     5.0          //电压循环检测次数
#define VOL_SCALE     4.333        //分压比例     10K/3K+1
#define LOW_VOLTAGE   9.6          //最低电压
#define BAT_RESOLUTION 0.05


typedef enum
{
	BATTERY_FULL,
	BATTERY_MEDIUM,
	BATTERY_LESS,
	BATTERY_LOW,
	BATTERY_ALARM
}Battery;


/**
 * @brief params of robot 
 */
typedef struct  Robot_Sensor_Params
{
    float      voltage;                 //real voltage = voltage/10
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;  
    float      acc[3];
    float      gyr[3];
#ifdef     imu_mag
    float      mag[3];
#endif
    float      elu[3];

}Robot_msgs; 

/**
 * @brief params of Motor 
 */
typedef struct  Motor_Controlled_Params
{
	float      velocity;    //
	float      yaw;         //
    int16_t    CtrlPulseL;
    int16_t    CtrlPulseR;
}Motor_msgs; 

/**
 * @brief 从IMU读取的原始数据
 */
typedef struct
{
	int16_t accRaw[3];
	int16_t gyrRaw[3];
#ifdef imu_mag
	int16_t magRaw;
#endif
	int16_t eluRaw[3];
}IMU_Raw_msg;

typedef struct
{
	float P;
	float I;
	float D;
}PID;

extern  Robot_msgs     robot_msgs;
extern  Motor_msgs     motor_msgs;
extern  IMU_Raw_msg    imu_raw_msg;
extern  PID            pid;
	

void voltage_check(void);
void oled_showContent(void);
void lowVoltageAlarm(void);


#endif
