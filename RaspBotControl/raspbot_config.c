#include "raspbot_config.h"

/**
  * @brief  ��ʱ��1(�߼���ʱ��)�ж�,����10ms�жϣ���Ϊ����Ŀ���ʱ���߼�
  * @param       
  */
void TIM1_IT_Init(u8 ms)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	NVIC_InitTypeDef NVIC_InitTypeStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //ʹ��APB2 TIM1
	
	
	TIM_TimeBaseInitTypeStruct.TIM_Period = ms*10-1;          // arr
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 7199;      // Tout=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision =TIM_CKD_DIV1; //�����벶��ʱ�˲��õĲ����Ƕ�ʱ���ķ�Ƶ��
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0;   //�߼���ʱ�����У��ظ����θ����ж�
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitTypeStruct);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);   //��������ж�
	
	NVIC_InitTypeStruct.NVIC_IRQChannel =TIM1_UP_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority =0;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority =1;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitTypeStruct);
	
	TIM_ClearFlag(TIM1,TIM_IT_Update);   
	TIM_Cmd(TIM1,ENABLE);
}

/**
  * @brief  ��ʱ��2(ͨ�ö�ʱ��),���ڿ�����Դ������
  * @param
  */
void TIM2_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    //ʹ��APB1 TIM2,PA1
	
	TIM_DeInit(TIM2);

	//���Ÿ���  PA1
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_1;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitTypeStruct);
	
	//��ʱ��
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;  //�޹�PWM���ã�ClockDivision�Ƕ�������ķ�Ƶ�������벶���ʱ��Ҫ�õ����൱���˲�
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period = 60000/1000-1;  
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =1199;          //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =0;           
	

	TIM_OC2Init(TIM2,&TIM_OCInitTypeStruct);  //CH2

	
	
	//�߼���ʱ������Ҫ���ã������޷����P��
//	TIM_CtrlPWMOutputs(TIMx,ENABLE);   //�߼���ʱ��  ʹ��ɲ���������Ĵ��� MOEʹ��

	//Ԥװ��ʹ�ܣ��޸ıȽ�ֵ������д��Ĵ���������ȴ���һ�����ڸ��ıȽ�ֵ
	//��P������û��Ӱ��
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���  CH2
	
	TIM_Cmd(TIM2,ENABLE);	
}
/**
  * @brief  ��ʱ��3(ͨ�ö�ʱ��)��1KHz,���PWM�����Ƶ��
  * @param       
  */
void TIM3_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   //ʹ��APB1 TIM3,PB0


	//���Ÿ��� PB0
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_0;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	//��ʱ��
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period =999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =71;          //1KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =500;           //ռ�ձ�
	

	TIM_OC3Init(TIM3,&TIM_OCInitTypeStruct);  //CH3

	
	
	//�߼���ʱ������Ҫ���ã������޷����P��
//	TIM_CtrlPWMOutputs(TIM3,ENABLE);   //�߼���ʱ��  ʹ��ɲ���������Ĵ��� MOEʹ��

	//Ԥװ��ʹ�ܣ��޸ıȽ�ֵ������д��Ĵ���������ȴ���һ�����ڸ��ıȽ�ֵ
	//��P������û��Ӱ��
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���  CH3

	
	TIM_Cmd(TIM3,ENABLE);	
}

/**
  * @brief  ��ʱ��4(ͨ�ö�ʱ��)��1KHz,���PWM�����Ƶ��
  * @param       
  */
void TIM4_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);   //ʹ��APB1 TIM4


	//���Ÿ���
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_8;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	//��ʱ��
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period =999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =71;          //10KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =500;           //ռ�ձ�
	
	TIM_OC3Init(TIM4,&TIM_OCInitTypeStruct);  //CH3

	//�߼���ʱ������Ҫ���ã������޷����P��
  //	TIM_CtrlPWMOutputs(TIMx,ENABLE);   //�߼���ʱ��  ʹ��ɲ���������Ĵ��� MOEʹ��

	//Ԥװ��ʹ�ܣ��޸ıȽ�ֵ������д��Ĵ���������ȴ���һ�����ڸ��ıȽ�ֵ
	//��P������û��Ӱ��
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���  CH3
	
	//Reset counter
  TIM_SetCounter(TIM4,0);
	
	TIM_Cmd(TIM4,ENABLE);	
}
/**
  * @brief  buzzer Ƶ��,���ڿ�����Դ������
  * @param  hz  1=<hz<=3000
  */
void buzzer_hz(u16 hz,FunctionalState enable)
{
	if(hz<1)
		hz=1;
	else if(hz>3000)
		hz=3000;
	u16 arr = 60000/hz -1;
	u16 pulse = 30000/hz -1;   //50%
	TIM_SetAutoreload(TIM2,arr);
	TIM_SetCompare2(TIM2,pulse);
	
	TIM_Cmd(TIM2,enable);
}


/**
  * @brief  ���IO��ʼ��  
  */
void motor_init(void)
{
	//���������ƶ˿�
	//���� A4 A5
	//�ҵ�� B9 C13
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2;		 		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOA,MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2);			
//	GPIO_PinLockConfig(GPIOA,MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_DIR_PIN1;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOB,MOTOR_R_DIR_PIN1);			
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_DIR_PIN2;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOC,MOTOR_R_DIR_PIN2);		

	//���PWM �˿�
	//�� PB0 TIM3_CH3
	//�� PB8 TIM4_CH3
//	TIM3_PWM_Init();
//	TIM4_PWM_Init();
}

/////////////////////////�������ת///////////////////////
void motor_L_stop(void)
{
	MOTOR_L_DIR1=0;
	MOTOR_L_DIR2=0;
}
void motor_L_forward(void)
{
	MOTOR_L_DIR1=1;
	MOTOR_L_DIR2=0;
}
void motor_L_reverse(void)
{
	MOTOR_L_DIR1=0;
	MOTOR_L_DIR2=1;
}

void motor_R_stop(void)
{
	MOTOR_R_DIR1=0;
	MOTOR_R_DIR2=0;

}
void motor_R_forward(void)
{
	MOTOR_R_DIR1=1;
	MOTOR_R_DIR2=0;
}
void motor_R_reverse(void)
{
	MOTOR_R_DIR1=0;
	MOTOR_R_DIR2=1;
}

void motor_forward(void)
{
	motor_L_forward();
	motor_R_forward();
}
void motor_reverse(void)
{
	motor_L_reverse();
	motor_R_reverse();
}
void motor_stop(void)
{
	motor_L_stop();
	motor_R_stop();
}

/**
  * @brief  ���PWM����
  * @param  duty_L������PWM <=1000   duty_R���ҵ��PWM <=1000      
  */
void motor_pwm(uint16_t duty_L,uint16_t duty_R)
{
	TIM_SetCompare3(TIM3,duty_L);
	TIM_SetCompare3(TIM4,duty_R);
}


/**
  * @brief  ������IO��ʼ��
  */
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);  //ʹ�ܶ�ʱ��4��ʱ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		

  
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;                    // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                   //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  ////TIM���ϼ���  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	//TIM_ICPolarity_Rising��ʾTIx���Բ�����     TIM_ICPolarity_Rising TIx���Է���(��ת�������½�)
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);   //ʹ�ñ�����ģʽ3
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
	TIM_ICStructInit(&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
  //Reset counter
  TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);
	
  TIM_Cmd(TIM3, ENABLE); 
	TIM_Cmd(TIM4, ENABLE); 
}
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM2 -> CNT=0 ;break;
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT; /* TIM3 -> CNT=0 */;break;	
	 }
	 return Encoder_TIM;
}
/**************************************************************************
�������ܣ�TIM3�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}

/**
  * @brief  ��ѹ����ʼ��
  */
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	ADC_InitTypeDef ADC_InitTypeStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  //ʹ��ADCʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    //ADCʱ�� 72/6=12MHZ,����14MHZ���Ȼ���
	ADC_DeInit(ADC1);                                    //��λʱ��
	
	//���� PB1
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AIN;                          //����ģ������
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	ADC_InitTypeStruct.ADC_Mode = ADC_Mode_Independent;                     //����ģʽ
	ADC_InitTypeStruct.ADC_ScanConvMode = DISABLE;                          //��ͨ��ģʽ
	ADC_InitTypeStruct.ADC_ContinuousConvMode = DISABLE;                    //����ת��
	ADC_InitTypeStruct.ADC_DataAlign = ADC_DataAlign_Right;                 //���� 
	ADC_InitTypeStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;    //������� 
	ADC_InitTypeStruct.ADC_NbrOfChannel = 1;                                //����ת����ADCͨ������Ŀ
	ADC_Init(ADC1,&ADC_InitTypeStruct);
		
	ADC_Cmd(ADC1,ENABLE);                          //ʹ��ADC
	ADC_ResetCalibration(ADC1);                    //����У׼
	while(ADC_GetResetCalibrationStatus(ADC1));   //�ȴ��������
	ADC_StartCalibration(ADC1);                    //��ʼУ׼ADC1;
	while(ADC_GetCalibrationStatus(ADC1));        //�ȴ�У׼���
	
}

u16 getAnalogValue()
{
	 //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		       //ʹ��ָ����ADC1�����ת����������	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));  //�ȴ�ת������
	return ADC_GetConversionValue(ADC1);
}

/**
	*		@brief  ����ͨ�ţ�
	*
	*
	*/

void UART1_Init(u32 baud)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    
}

/**
*
*
*/
void UART2_Init(u32 baud)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART2_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART2_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    
}



/**
  * @brief  board IO initial
  */
void board_configInit(void)
{
	//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
<<<<<<< HEAD
=======
	
	
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee
	
	
	
//	SystemInit();
	delay_init();
	
	oled_init();
	oled_picture(0,0,128,64,start_bmp);
	oled_update();
	delay_ms(1000);
	OLED_Clear();
	
	motor_init();
<<<<<<< HEAD
	UART1_Init(115200);
	TIM1_IT_Init(10);         /* must initialize after usart,because send data in interrupts */
=======
	TIM1_IT_Init(10);
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee
	TIM2_PWM_Init();
	adc_init();
	
//	motor_stop();
//	motor_pwm(500,500);
//	encoder_init();
	
	
	
	
}
