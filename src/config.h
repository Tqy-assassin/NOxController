/*
 * config.h
 *
 *  Created on: 2019年12月5日
 *      Author: tianqingyuan
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION	"NN0V04.65A"
//#define VOLTAGE_COMPATIBILITY				//电压管理策略
#ifndef VOLTAGE_COMPATIBILITY
#define WORKVOLAGE12Vor24V	0		//12V : 1 | 24V : 0
#endif

#define ONSEMI_SCHEME 1    //仿NTK: 0 | 原版 :1
#define NTK_KIND	1//250
#define BENZ_KIND	2//500
#define DEFAULT		NTK_KIND

#define VENDOR_ID	NTK_KIND
//#define DBFLAG 1
//#define Default_SourceAddr		//J1939_Addr: SourceAddrATI1 | SourceAddrATO1 | SourceAddrATI2 | SourceAddrATO2

//#define JUDGE_RECEIVE	//开启CAN_Receive地址判断，只接收广播信息或目的地址为自己的信息

//#define DEBUG
//#define UART_CONTROL	//UART显示+控制

#ifdef UART_CONTROL
//#define CURSOR_CONTROL	//开启串口打印中的光标控制（"可能"是导致鼠标乱跳的元凶）
#endif

#define AvgFilter	1
#define LowFilter	2
#define DF2Filter	3
#define Filter_type AvgFilter

#define BUS_CLK_HZ		24000000		//总线频率
#define SRC_BUS_CLK_HZ	48000000		//主频

#define VREF0_METHOD	2	//0: VComm控制Vref0 ; 1: Ip1控制Vref0 ; 2: 查表预控制+Ip1微小控制Vref0
#define VREF1_METHOD	0   //0: Vref0固定  ；1：查表

#define O2CAVGTIME		30
#define NOxCAVGTIME 	80
#define Vref0AVGTIME 	10
#define Vref1AVGTIME 	10
#define NOxAVGTIME 		10

#define NORAML_LAUNCH_ENABLE 1	//使能常规方式启动控制器
#endif /* CONFIG_H_ */
