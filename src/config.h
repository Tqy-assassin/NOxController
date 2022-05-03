/*
 * config.h
 *
 *  Created on: 2019閿熸枻鎷�12閿熸枻鎷�5閿熸枻鎷�
 *      Author: tianqingyuan
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define NOXADDR_AUTO				0
#define NOXADDR_ATI1				1
#define NOXADDR_ATO1				2
#define NOXADDR_ATI2				3
#define NOXADDR_ATO2				4

#define NOXSOURCEADDR				NOXADDR_AUTO 			//Sensor address----NOXADDR_AUTO:Automatic address recognition; NOXADDR_ATI1:Nitrogen oxygen before; NOXADDR_ATO1:After nitrogen oxygen

#define VERSION	"GXAV05.06A"
// #define MANUFACTURERSNAME	"xxxxxxV05.06A"     //锟酵伙拷锟斤拷锟斤拷

#define VOLTAGE_COMPATIBILITY				//閿熸枻鎷峰帇閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
#ifndef VOLTAGE_COMPATIBILITY
#define WORKVOLAGE12Vor24V	0		//12V : 1 | 24V : 0
#endif

#define ONSEMI_SCHEME 0    //閿熸枻鎷種TK: 0 | 鍘熼敓鏂ゆ嫹 :1
#define NTK_KIND	1//250
#define BENZ_KIND	2//500
#define GWM_KIND	3//500
#define YUC_Y24		4//500
#define DEFAULT		NTK_KIND

#define VENDOR_ID	NTK_KIND
//#define DBFLAG 1
//#define Default_SourceAddr		//J1939_Addr: SourceAddrATI1 | SourceAddrATO1 | SourceAddrATI2 | SourceAddrATO2

//#define JUDGE_RECEIVE	//閿熸枻鎷烽敓鏂ゆ嫹CAN_Receive閿熸枻鎷峰潃閿熷彨鏂綇鎷峰彧閿熸枻鎷烽敓绉稿箍鎾敓鏂ゆ嫹鎭敓鏂ゆ嫹鐩敓渚ョ鎷峰潃涓洪敓鐨嗙》鎷烽敓鏂ゆ嫹閿熸枻鎷锋伅

//#define DEBUG
//#define UART_CONTROL	//UART閿熸枻鎷风ず+閿熸枻鎷烽敓鏂ゆ嫹

#ifdef UART_CONTROL
//#define CURSOR_CONTROL	//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鑺傝揪鎷峰嵃閿熷彨鐨勭櫢鎷烽敓鏂ゆ嫹閿熺嫛锝忔嫹"閿熸枻鎷烽敓鏂ゆ嫹"閿熻纰夋嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鐨嗩亷鎷风閿燂拷
#endif

#define AvgFilter	1
#define LowFilter	2
#define DF2Filter	3
#define Filter_type AvgFilter

#define BUS_CLK_HZ		24000000		//閿熸枻鎷烽敓鏂ゆ嫹棰戦敓鏂ゆ嫹
#define SRC_BUS_CLK_HZ	48000000		//閿熸枻鎷烽

#define VREF0_METHOD	1	//0: VComm閿熸枻鎷烽敓鏂ゆ嫹Vref0 ; 1: Ip1閿熸枻鎷烽敓鏂ゆ嫹Vref0 ; 2: 閿熸枻鎷烽敓鐨嗐倧鎷烽敓鏂ゆ嫹閿燂拷+Ip1寰皬閿熸枻鎷烽敓鏂ゆ嫹Vref0
#define VREF1_METHOD	0   //0: Vref0閿熸暀璁规嫹  閿熸枻鎷�1閿熸枻鎷烽敓鏂ゆ嫹閿燂拷

#define O2CAVGTIME		10
#define NOxCAVGTIME 	80
#define Vref0AVGTIME 	10
#define Vref1AVGTIME 	10
#define NOxAVGTIME 		10

#define NORAML_LAUNCH_ENABLE 1	//浣块敓鏉扮鎷烽敓鑺ユ柟寮忛敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
#endif /* CONFIG_H_ */
