/*
 * common.h
 *
 *  Created on: 2019年7月11日
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
#define CLEAR() printf("\033[2J")					//清屏
#define MOVEUP(x) printf("\033[%dA", (x))
#define MOVEDOWN(x) printf("\033[%dB", (x))
#define MOVELEFT(y) printf("\033[%dD", (y))
#define MOVERIGHT(y) printf("\033[%dC",(y))
#define RESET_CURSOR() printf("\033[H")				//设置：重置光标至原点
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))	//设置：移动光标至x行y列
#define CLEARLINE() printf("\033[K")				//设置：清楚从光标到行尾的字符
#define SAVEPOS() printf("\033[s")					//设置：保存光标位置
#define LODEPOS() printf("\033[u")					//设置：恢复光标位置
#define RESET() printf("\033[0m")					//设置：清楚所有设置
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
void DelayUS(uint32_t u32TimeUS);		//阻塞延时函数 单位：US
//void DelayMS(uint32_t u32TimeMS);		//阻塞延时函数 单位：MS
//void DelayS(uint32_t u32TimeS);  		//阻塞延时函数 单位：S

//#ifdef DEBUG
void printstring(char* string);			//串口打印字符串
void printfloat(float value);			//串口打印浮点数
void printHex(unsigned int val);
//#endif

void sysinit(void);				//系统初始化	配置频率和串口
void UART0_ISR(void);
void Parse_UartCmd(void);

#endif /* COMMON_H_ */
