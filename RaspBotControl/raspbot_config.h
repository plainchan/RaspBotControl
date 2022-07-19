#ifndef __RASPBOT_CONFIG_H__
#define __RASPBOT_CONFIG_H__

#include "sys.h"
#include "oled.h"
#include "delay.h"


#define STATE_LED            PCout(13)

#define MOTOR_EN_L           PAout(4)
#define MOTOR_EN_R           PBout(2)

#define MOTOR_EN_L_PIN       GPIO_Pin_4       //A4
#define MOTOR_EN_R_PIN       GPIO_Pin_2       //B2

#define  M_PI    3.1415926

//编码器参数
#define reduction_Ratio                  30       // 电机减速比
#define encoder_line                     13       // 编码器线数
#define multiplier_factor                4        // A/B相倍频因子
#define PPR                   (reduction_Ratio*encoder_line*multiplier_factor)
//小车参数
#define wheelTrack                     0.203f       //轮距
#define wheelRadius                    0.0325       //轮胎半径

//定时器间隔
#define intervalTimer                   0.01f

extern volatile char safe_mode;
extern volatile char uart_lock;
           

void board_configInit(void);
void motor_pwm(int16_t duty_L,int16_t duty_R);
u16 getAnalogValue(void);
short Read_Encoder(u8 TIMX);
void security_mode(void);

#endif
