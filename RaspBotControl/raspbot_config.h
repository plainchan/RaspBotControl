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

           

void board_configInit(void);
void motor_pwm(int16_t duty_L,int16_t duty_R);
u16 getAnalogValue(void);
int Read_Encoder(u8 TIMX);

#endif
