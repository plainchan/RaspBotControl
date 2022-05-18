#ifndef __RASPBOT_CONFIG_H__
#define __RASPBOT_CONFIG_H__

#include "sys.h"
#include "oled.h"
#include "delay.h"

#define MOTOR_L_PWM_PIN      GPIO_Pin_0       //B0
#define MOTOR_L_DIR_PIN1     GPIO_Pin_4       //A4
#define MOTOR_L_DIR_PIN2     GPIO_Pin_5       //A5
#define MOTOR_L_DIR1         PAout(4)          
#define MOTOR_L_DIR2				 PAout(5)          

#define MOTOR_R_PWM_PIN      GPIO_Pin_8       //B0
#define MOTOR_R_DIR_PIN1     GPIO_Pin_9       //B9
#define MOTOR_R_DIR_PIN2     GPIO_Pin_13      //C13
#define MOTOR_R_DIR1         PBout(9)          
#define MOTOR_R_DIR2         PCout(13)         

void board_configInit(void);
void motor_pwm(uint16_t duty_L,uint16_t duty_R);
u16 getAnalogValue(void);
int Read_Encoder(u8 TIMX);

#endif
