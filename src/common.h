/*
 * common.h
 *
 *  Created on: 2019锟斤拷7锟斤拷11锟斤拷
 *      Author: tianqingyuan
 */
#ifndef COMMON_H_
#define COMMON_H_

#include "derivative.h"
#include "config.h"
#include "clock.h"

#ifndef FALSE
#define FALSE	(0)
#endif
#ifndef TRUE
#define TRUE	(1)
#endif

#ifdef DEBUG
#define CLEAR() printf("\033[2J")					//锟斤拷锟斤拷
#define MOVEUP(x) printf("\033[%dA", (x))
#define MOVEDOWN(x) printf("\033[%dB", (x))
#define MOVELEFT(y) printf("\033[%dD", (y))
#define MOVERIGHT(y) printf("\033[%dC",(y))
#define RESET_CURSOR() printf("\033[H")				//锟斤拷锟矫ｏ拷锟斤拷锟矫癸拷锟斤拷锟皆拷锟�
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))	//锟斤拷锟矫ｏ拷锟狡讹拷锟斤拷锟斤拷锟絰锟斤拷y锟斤拷
#define CLEARLINE() printf("\033[K")				//锟斤拷锟矫ｏ拷锟斤拷锟斤拷庸锟疥到锟斤拷尾锟斤拷锟街凤拷
#define SAVEPOS() printf("\033[s")					//锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟斤拷位锟斤拷
#define LODEPOS() printf("\033[u")					//锟斤拷锟矫ｏ拷锟街革拷锟斤拷锟轿伙拷锟�
#define RESET() printf("\033[0m")					//锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
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

typedef struct{
	union{
		struct{
			uint8_t byte0Data;
			uint8_t byte1Data;
			uint8_t byte2Data;
			uint8_t byte3Data;
			uint8_t byte4Data;
			uint8_t byte5Data;
			uint8_t byte6Data;
			uint8_t byte7Data;
		}TxFrame;
		uint8_t TxData[8];
	};
}GeneralTFrame_TypeDef;

uint8_t Hysteresis_Comparator(int16_t input,int16_t lower,int16_t upper);
void DelayUS(uint32_t u32TimeUS);		//锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷 锟斤拷位锟斤拷US
//void DelayMS(uint32_t u32TimeMS);		//锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷 锟斤拷位锟斤拷MS
//void DelayS(uint32_t u32TimeS);  		//锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷 锟斤拷位锟斤拷S

//#ifdef DEBUG
void printstring(char* string);			//锟斤拷锟节达拷印锟街凤拷锟斤拷
void printfloat(float value);			//锟斤拷锟节达拷印锟斤拷锟斤拷锟斤拷
void printHex(unsigned int val);
//#endif

void sysinit(void);				//系统锟斤拷始锟斤拷	锟斤拷锟斤拷频锟绞和达拷锟斤拷
void UART0_ISR(void);
void Parse_UartCmd(void);

#endif /* COMMON_H_ */
