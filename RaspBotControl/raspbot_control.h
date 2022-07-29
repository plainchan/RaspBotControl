#ifndef __RASPBOT_CONTROL_H__
#define __RASPBOT_CONTROL_H__

#include "stdint.h"

//#define imu_mag

#define  G  9.780

#define ANALOG_VOL    3.30          //模拟电压参考值
#define COV_COUNT     100         	//电压循环检测次数
#define VOL_SCALE     4.333         //分压比例     10K/3K+1
#define LOW_VOLTAGE   9.6           //最低电压
#define BAT_RESOLUTION 0.05

#define MAX_SPEED      1.2
#define MAX_STEERING   2


#define  ATTRIBUTION_JOYSTICK_SPEED       0
#define  ATTRIBUTION_COMPUTER_SPEED       1
#define  ATTRIBUTION_DISCONNECT_SPEED     2
#define  ATTRIBUTION_NONE_SPEED           3


#define joy_forward_scale    0.003f
#define joy_backward_scale   0.0015f
#define joy_steering_scale   0.005f

#define  STATE_LED_NORMAL_DURATION   500

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
 * @brief msgs of Motor  from computer or joystick
 */
typedef struct  Motor_Controlled_Params
{
	float      velocity;        //
	float      angular;         //
	uint8_t    attribution;
}Motor_msgs; 

/**
 * @brief imu data from register
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
	float Kp;
	float Ki;
	float Kd;

	float error[2];
	float last_error[2];
	float accumu_error[2];
	
	int16_t limMax;
	int16_t limMin;

	int16_t out_pwm[2];
}PID;

extern  Robot_msgs     robot_msgs;
extern  Motor_msgs     motor_msgs;
extern  IMU_Raw_msg    imu_raw_msg;
extern  PID            pid;
	

extern volatile char safe_mode;

void voltage_check(void);
void oled_showContent(void);
void lowVoltageAlarm(void);
void ps2_control(void);
void param_init(void);
#endif
