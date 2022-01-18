/*
 * config.h
 *
 *  Created on: 2019锟斤拷12锟斤拷5锟斤拷
 *      Author: tianqingyuan
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION	"GXAV05.06A"
#define VOLTAGE_COMPATIBILITY				//锟斤拷压锟斤拷锟斤拷锟斤拷锟�
#ifndef VOLTAGE_COMPATIBILITY
#define WORKVOLAGE12Vor24V	0		//12V : 1 | 24V : 0
#endif

#define ONSEMI_SCHEME 0    //锟斤拷NTK: 0 | 原锟斤拷 :1
#define NTK_KIND	1//250
#define BENZ_KIND	2//500
#define GWM_KIND	3//500
#define DEFAULT		NTK_KIND

#define VENDOR_ID	GWM_KIND
//#define DBFLAG 1
//#define Default_SourceAddr		//J1939_Addr: SourceAddrATI1 | SourceAddrATO1 | SourceAddrATI2 | SourceAddrATO2

//#define JUDGE_RECEIVE	//锟斤拷锟斤拷CAN_Receive锟斤拷址锟叫断ｏ拷只锟斤拷锟秸广播锟斤拷息锟斤拷目锟侥碉拷址为锟皆硷拷锟斤拷锟斤拷息

//#define DEBUG
//#define UART_CONTROL	//UART锟斤拷示+锟斤拷锟斤拷

#ifdef UART_CONTROL
//#define CURSOR_CONTROL	//锟斤拷锟斤拷锟斤拷锟节达拷印锟叫的癸拷锟斤拷锟狡ｏ拷"锟斤拷锟斤拷"锟角碉拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆拷祝锟�
#endif

#define AvgFilter	1
#define LowFilter	2
#define DF2Filter	3
#define Filter_type AvgFilter

#define BUS_CLK_HZ		24000000		//锟斤拷锟斤拷频锟斤拷
#define SRC_BUS_CLK_HZ	48000000		//锟斤拷频

#define VREF0_METHOD	1	//0: VComm锟斤拷锟斤拷Vref0 ; 1: Ip1锟斤拷锟斤拷Vref0 ; 2: 锟斤拷锟皆わ拷锟斤拷锟�+Ip1微小锟斤拷锟斤拷Vref0
#define VREF1_METHOD	0   //0: Vref0锟教讹拷  锟斤拷1锟斤拷锟斤拷锟�

#define O2CAVGTIME		10
#define NOxCAVGTIME 	80
#define Vref0AVGTIME 	10
#define Vref1AVGTIME 	10
#define NOxAVGTIME 		10

#define NORAML_LAUNCH_ENABLE 1	//使锟杰筹拷锟芥方式锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
#endif /* CONFIG_H_ */
