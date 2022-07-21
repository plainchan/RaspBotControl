#include "raspbot_config.h"
#include "ps2lib.h"
/**
 * @brief  ��ʱ��1(�߼���ʱ��)�ж�,����10ms�жϣ���Ϊ����Ŀ���ʱ���߼�
 * @param
 */
void TIM1_IT_Init(u8 ms)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ��APB2 TIM1

	TIM_TimeBaseInitTypeStruct.TIM_Period = ms * 10 - 1;		 // arr
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 7199;			 // Tout=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1; //�����벶��ʱ�˲��õĲ����Ƕ�ʱ���ķ�Ƶ��
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0; //�߼���ʱ�����У��ظ����θ����ж�
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitTypeStruct);

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //��������ж�

	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitTypeStruct);

	TIM_ClearFlag(TIM1, TIM_IT_Update);
	TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  ��ʱ��2(ͨ�ö�ʱ��),������ģʽ��
 * @param
 */
void TIM2_Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
//	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); //ʹ�ܶ�ʱ��4��ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;				// Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;					//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////TIM���ϼ���
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// TIM_ICPolarity_Rising��ʾTIx���Բ�����     TIM_ICPolarity_Rising TIx���Է���(��ת�������½�)
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3

	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update); //���TIM�ĸ��±�־λ
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

//	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitTypeStruct);

	// Reset counter
	TIM_SetCounter(TIM2, 0);

	TIM_Cmd(TIM2, ENABLE);
}
/**
 * @brief  ��ʱ��4(ͨ�ö�ʱ��)��������ģʽ
 * @param
 */
void TIM4_Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
//	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE); //ʹ�ܶ�ʱ��4��ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;				// Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;					//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// TIM_ICPolarity_Rising��ʾTIx���Բ�����     TIM_ICPolarity_Rising TIx���Է���(��ת�������½�)
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3


	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update); //���TIM�ĸ��±�־λ
//	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

//	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM4_IRQn;
//	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitTypeStruct);

	// Reset counter
	TIM_SetCounter(TIM4, 0);

	TIM_Cmd(TIM4, ENABLE);
}
/**
 * @brief  ��������ʼ��
 */
void encoder_init(void)
{
	TIM2_Encoder_Init();
	TIM4_Encoder_Init();
}

/**
 * @brief  ������IO��ʼ��
 */
short Read_Encoder(u8 TIMX)
{
	short pluse;
	switch (TIMX)
	{
	case 2:
		pluse = (short)TIM2->CNT;
		TIM2->CNT = 0;
		break;
	case 4:
		pluse = (short)TIM4->CNT;
		TIM4->CNT = 0;
		break;
	}
	return pluse;
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ��APB1 TIM3,PB0
	
	//���Ÿ��� CH1 PA6  CH2 PA7
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;   
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeStruct);
	//���Ÿ��� CH3 PB0  CH4 PB1
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;   
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);
	
	
	//��ʱ��
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period = 999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 71; // 1KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitTypeStruct);

	// PWM
	TIM_OCInitTypeStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse = 0; //ռ�ձ�
	
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStruct); // CH1
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStruct); // CH2
	TIM_OC3Init(TIM3, &TIM_OCInitTypeStruct); // CH3
	TIM_OC4Init(TIM3, &TIM_OCInitTypeStruct); // CH4

	//�߼���ʱ������Ҫ���ã������޷����P��
	//	TIM_CtrlPWMOutputs(TIM3,ENABLE);   //�߼���ʱ��  ʹ��ɲ���������Ĵ��� MOEʹ��

	//Ԥװ��ʹ�ܣ��޸ıȽ�ֵ������д��Ĵ���������ȴ���һ�����ڸ��ıȽ�ֵ
	//��P������û��Ӱ��
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���  CH3

	TIM_Cmd(TIM3, ENABLE);
}



////////////////////////
void MOTOR_L_ENABLE(FunctionalState state)
{
	if(state)
		MOTOR_EN_L=1;
	else
		MOTOR_EN_L=0;
}
void MOTOR_R_ENABLE(FunctionalState state)
{
	if(state)
		MOTOR_EN_R=1;
	else
		MOTOR_EN_R=0;
}
void MOTOR_ENABLE(FunctionalState state)
{
	MOTOR_L_ENABLE(state);
	MOTOR_R_ENABLE(state);
}
/**
 * @brief  ���IO��ʼ��
 */
void motor_init(void)
{
	//���ʹ�ܿ��ƶ˿�
	//���� A4
	//�ҵ�� B2
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = MOTOR_EN_L_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA, MOTOR_EN_L_PIN);
	
	GPIO_InitStructure.GPIO_Pin =MOTOR_EN_R_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOB,MOTOR_EN_R_PIN);
	
	//	GPIO_PinLockConfig(GPIOA,MOTOR_EN_L_PIN);
	TIM3_PWM_Init();
	MOTOR_ENABLE(ENABLE);

}



/**
 * @brief  ���PWM����
 * @param  duty_L������PWM <=1000   duty_R���ҵ��PWM <=1000  
 */
void motor_pwm(int16_t duty_L, int16_t duty_R)
{
	if(duty_L>=0)
	{	
		TIM_SetCompare1(TIM3, duty_L);
		TIM_SetCompare2(TIM3, 0);
	}
	else
	{
		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, -duty_L);
	}
	
	if(duty_R>=0)
	{	
		TIM_SetCompare3(TIM3, 0);
		TIM_SetCompare4(TIM3, duty_R);
	}
	else
	{
		TIM_SetCompare3(TIM3, -duty_R);
		TIM_SetCompare4(TIM3, 0);
	}
}


/**
 * @brief  TIM2 �жϷ�����������������жϴ���
 */
void TIM2_IRQHandler(void)
{

	if (TIM2->SR & 0X0001) //����ж�
	{
	}
	TIM2->SR &= ~(1 << 0); //����жϱ�־λ
}
/**
 * @brief  TIM4 �жϷ�����������������жϴ���
 */
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & 0X0001) //����ж�
	{
	}
	TIM4->SR &= ~(1 << 0); //����жϱ�־λ
}

/**
 * @brief  ��ѹ����ʼ��
 */
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	ADC_InitTypeDef ADC_InitTypeStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADCʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADCʱ�� 72/6=12MHZ,����14MHZ���Ȼ���
	ADC_DeInit(ADC1);				  //��λʱ��

	//���� PA5
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AIN; //����ģ������
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_5;
	//	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);

	ADC_InitTypeStruct.ADC_Mode = ADC_Mode_Independent;					 //����ģʽ
	ADC_InitTypeStruct.ADC_ScanConvMode = DISABLE;						 //��ͨ��ģʽ
	ADC_InitTypeStruct.ADC_ContinuousConvMode = DISABLE;				 //����ת��
	ADC_InitTypeStruct.ADC_DataAlign = ADC_DataAlign_Right;				 //����
	ADC_InitTypeStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�������
	ADC_InitTypeStruct.ADC_NbrOfChannel = 1;							 //����ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitTypeStruct);

	ADC_Cmd(ADC1, ENABLE);		//ʹ��ADC
	ADC_ResetCalibration(ADC1); //����У׼
	while (ADC_GetResetCalibrationStatus(ADC1))
		;						//�ȴ��������
	ADC_StartCalibration(ADC1); //��ʼУ׼ADC1;
	while (ADC_GetCalibrationStatus(ADC1))
		; //�ȴ�У׼���
}

u16 getAnalogValue()
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); // ADC1,ADCͨ��,����ʱ��Ϊ239.5����

	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ��ָ����ADC1�����ת����������
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
		; //�ȴ�ת������
	return ADC_GetConversionValue(ADC1);
}

/**
 *		@brief  ����ͨ�ţ�
 *
 *
 */

void UART1_Init(u32 baud)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ��USART1��GPIOAʱ��

	// USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��GPIOA.9

	// USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  // PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //��ʼ��GPIOA.10

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //����ָ���Ĳ�����ʼ��VIC�Ĵ���

	// USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;										//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure);	   //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);
}


void UART2_Init(u32 baud)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ��USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   //��ʼ��GPIOA.2

	// USART2_RX	  GPIOA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;			  // PA.3
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPD;   //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //��ʼ��GPIOA.3

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //����ָ���Ĳ�����ʼ��VIC�Ĵ���

	// USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;										//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

	USART_Init(USART2, &USART_InitStructure);	   //��ʼ������1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);
}

/**
 *
 *
 */
void UART3_Init(u32 baud)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);   //ʹ��USART1��GPIOAʱ��

	// USAR3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��GPIOB.10

	// UART3_RX	  GPIOB.11��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			// PB.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //����
	GPIO_Init(GPIOB, &GPIO_InitStructure);				  //��ʼ��GPIOB.11

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //����ָ���Ĳ�����ʼ��VIC�Ĵ���

	// USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;										//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

	USART_Init(USART3, &USART_InitStructure);	   //��ʼ������1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
	USART_Cmd(USART3, ENABLE);
}

/**
 * @brief  ״ָ̬ʾ��
 */
void stateLED_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//  GPIOC.13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         // PC.13
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);			      //��ʼ��GPIOC.13
}

/**
 * @brief  ��ȫģʽ��������������/����
 */
volatile char safe_mode = 0;
void security_mode(void)
{
	while(safe_mode)
	{
	}
}


volatile char uart_lock = 0;

/**
 * @brief  board IO initial
 */
void board_configInit(void)
{
	SystemInit();
	//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/* Disable JLink, enable SW */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);

	//�жϷ���
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	
	/* ps2 port use JTAG */
	delay_init();
	ps2_init();
	stateLED_init();
	oled_init();
	oled_picture(0, 0, 128, 64, start_bmp);
	oled_update();
	
	delay_ms(1000);
	OLED_Clear();
	
	
	motor_init();
	UART1_Init(115200);    // ͨ�Ŵ���
//	UART2_Init(115200);    // IMU����
//	UART3_Init(115200);    // ����3�����˿�

	TIM1_IT_Init(10); /* must initialize after usart,because send data in interrupts */
	
	motor_init();
	adc_init();
	encoder_init();
	
	//Due to the occupation of JTAG's ports,must configure the remapping of IO.
	//It can't work if the position of the function "ps2_init()" ahead any sentens int the function "board_configInit";
	// So the function "ps2_init()" must be in the end.
	//WHY???????
	ps2_init();
	
}
