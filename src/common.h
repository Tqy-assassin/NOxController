/*
 * common.h
 *
 *  Created on: 2019��7��11��
 *      Author: tianqingyuan
 */
#include "derivative.h"
#include "config.h"
#include "clock.h"

#ifndef COMMON_H_
#define COMMON_H_

#ifndef FALSE
#define FALSE	(0)
#endif
#ifndef TRUE
#define TRUE	(1)
#endif

#ifdef DEBUG
#define CLEAR() printf("\033[2J")					//����
#define MOVEUP(x) printf("\033[%dA", (x))
#define MOVEDOWN(x) printf("\033[%dB", (x))
#define MOVELEFT(y) printf("\033[%dD", (y))
#define MOVERIGHT(y) printf("\033[%dC",(y))
#define RESET_CURSOR() printf("\033[H")				//���ã����ù����ԭ��
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))	//���ã��ƶ������x��y��
#define CLEARLINE() printf("\033[K")				//���ã�����ӹ�굽��β���ַ�
#define SAVEPOS() printf("\033[s")					//���ã�������λ��
#define LODEPOS() printf("\033[u")					//���ã��ָ����λ��
#define RESET() printf("\033[0m")					//���ã������������
#else
#define CLEAR()
#define MOVEUP(x)
#define MOVEDOWN(x)
#define MOVELEFT(y)
#define MOVERIGHT(y)
#define RESET_CURSOR()
#define MOVETO(x,y)
#define CLEARLINE()
#define SAVEPOS()
#define LODEPOS()
#define RESET()
#endif


#define myabs(a)	((a)<0?-(a):(a))
#define range(a,x) ((a)<(-x)?(-x):((a)>(x)?(x):(a)))		//x>0

uint8_t Hysteresis_Comparator(int16_t input,int16_t lower,int16_t upper);
void DelayUS(uint32_t u32TimeUS);		//������ʱ���� ��λ��US
//void DelayMS(uint32_t u32TimeMS);		//������ʱ���� ��λ��MS
//void DelayS(uint32_t u32TimeS);  		//������ʱ���� ��λ��S

//#ifdef DEBUG
void printstring(char* string);			//���ڴ�ӡ�ַ���
void printfloat(float value);			//���ڴ�ӡ������
void printHex(unsigned int val);
//#endif

void sysinit(void);				//ϵͳ��ʼ��	����Ƶ�ʺʹ���
void UART0_ISR(void);
void Parse_UartCmd(void);

#endif /* COMMON_H_ */
