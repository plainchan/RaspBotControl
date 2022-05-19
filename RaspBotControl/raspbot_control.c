#include "raspbot_control.h"
#include "raspbot_config.h"

#include "sys.h"


/***** С�������ṹ��  *****/
Robot_msgs                robot_msgs={0};

volatile float power_voltage=11.1;

int  receiveFlag=0;

Battery battery=BATTERY_FULL;


void voltage_check(void)
{
	u32 value=0;
	for(int i=1;i<=COV_COUNT;++i)
	{
		value+=getAnalogValue();
	}
	float voltage = ((value/COV_COUNT)/4096.0)*ANALOG_VOL*VOL_SCALE;
	//���µ�ѹ
	if(voltage-power_voltage>BAT_RESOLUTION || power_voltage-voltage>BAT_RESOLUTION)
		power_voltage = voltage;

	if(power_voltage<=LOW_VOLTAGE)
		battery=BATTERY_ALARM;	
	else if(power_voltage>11.85)
		battery=BATTERY_FULL;
	else if(power_voltage>11.1 && power_voltage<=11.85)
		battery=BATTERY_MEDIUM;
	else if(power_voltage>10.35 && power_voltage<=11.1)
		battery=BATTERY_LESS;
	else if(power_voltage>9.6 && power_voltage<=10.35)
		battery=BATTERY_LOW;
}

void lowVoltageAlarm(void)
{

//	TIM2_PWM_Init(50);
	while(battery==BATTERY_ALARM)
	{
		oled_picture(40,20,48,24,LOW_BATTERY);
		oled_update();
		delay_ms(500);
		OLED_Clear();
		delay_ms(200);
	}
}


void oled_showContent(void)
{
	//��ʾ��ѹ
	oled_float(74,0,power_voltage,1,12);
	oled_char(98,0,'V',12,1);
	switch(battery)
	{
		case BATTERY_FULL:oled_picture(104,0,24,12,BATTERY_4); break;
		case BATTERY_MEDIUM:oled_picture(104,0,24,12,BATTERY_3); break;
		case BATTERY_LESS:oled_picture(104,0,24,12,BATTERY_2); break;
		case BATTERY_LOW:oled_picture(104,0,24,12,BATTERY_1); break;
		case BATTERY_ALARM:lowVoltageAlarm(); break;
	}
	
	
//   //��ʾ�ٶ�
// 	if(receiveFlag==1)
// 	{
// 		float v = stream_msgs.speed_msgs.velocity/1000.0;
// 		float y = stream_msgs.speed_msgs.yaw/1000.0;
// 		if(v>0) oled_char(98,36,'+',12,1); else oled_char(98,36,'-',12,1);
// 		if(y>0) oled_char(98,52,'+',12,1); else oled_char(98,52,'-',12,1);
// 		oled_float(104,36,v,2,12);
// 		oled_float(104,52,y,2,12);
// //		oled_num(0,0,sizeof(Frame_Robot_msg),3,12);
// 	}
	


	oled_update();
}















/**
  * @brief  ��ʱ��1�ж���ڣ�10ms�ж�
	*         ��������߼������Ƶ������������
  */
void TIM1_UP_IRQHandler(void)
{
	//�����־λ
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	MOTOR_L_DIR1 = !MOTOR_L_DIR1;
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
//	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET) // ��� ORE ��־
//  {
//		USART_ReceiveData(USART1);
//  }
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		receiveFlag = parse_stream(&stream_msgs,USART_ReceiveData(USART1));
  } 
} 


void test(void)
{
	//������ͨ�Ų���
	sendFrame_Encoder_dpkg(&frame_encode_dpkg,244);
}


