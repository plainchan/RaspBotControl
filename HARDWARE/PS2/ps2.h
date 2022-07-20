#ifndef __PS2_H__
#define __PS2_H__
#include "delay.h"
#include "sys.h"



// #define DI   PBin(12)           //PB12  ����

// #define DO_H PBout(13)=1        //����λ��
// #define DO_L PBout(13)=0        //����λ��

// #define CS_H PBout(14)=1       //CS����
// #define CS_L PBout(14)=0       //CS����

// #define CLK_H PBout(15)=1      //ʱ������
// #define CLK_L PBout(15)=0      //ʱ������

#define DI   PAin(15)           //PB12  ����

#define DO_H PBout(3)=1        //����λ��
#define DO_L PBout(3)=0        //����λ��

#define CS_H PBout(4)=1       //CS����
#define CS_L PBout(4)=0       //CS����

#define CLK_H PBout(5)=1      //ʱ������
#define CLK_L PBout(5)=0      //ʱ������


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8



extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);//�ж��Ƿ�Ϊ���ģ�?
void PS2_ReadData(void);
void PS2_Cmd(u8 cmd);		  //
u8 PS2_DataKey(void);		  //��ֵ��ȡ
u8 PS2_AnologData(u8 button); //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  //������ݻ�����?

#endif





