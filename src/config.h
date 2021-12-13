/*
 * config.h
 *
 *  Created on: 2019��12��5��
 *      Author: tianqingyuan
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION	"NN0V04.65A"
//#define VOLTAGE_COMPATIBILITY				//��ѹ�������
#ifndef VOLTAGE_COMPATIBILITY
#define WORKVOLAGE12Vor24V	0		//12V : 1 | 24V : 0
#endif

#define ONSEMI_SCHEME 1    //��NTK: 0 | ԭ�� :1
#define NTK_KIND	1//250
#define BENZ_KIND	2//500
#define DEFAULT		NTK_KIND

#define VENDOR_ID	NTK_KIND
//#define DBFLAG 1
//#define Default_SourceAddr		//J1939_Addr: SourceAddrATI1 | SourceAddrATO1 | SourceAddrATI2 | SourceAddrATO2

//#define JUDGE_RECEIVE	//����CAN_Receive��ַ�жϣ�ֻ���չ㲥��Ϣ��Ŀ�ĵ�ַΪ�Լ�����Ϣ

//#define DEBUG
//#define UART_CONTROL	//UART��ʾ+����

#ifdef UART_CONTROL
//#define CURSOR_CONTROL	//�������ڴ�ӡ�еĹ����ƣ�"����"�ǵ������������Ԫ�ף�
#endif

#define AvgFilter	1
#define LowFilter	2
#define DF2Filter	3
#define Filter_type AvgFilter

#define BUS_CLK_HZ		24000000		//����Ƶ��
#define SRC_BUS_CLK_HZ	48000000		//��Ƶ

#define VREF0_METHOD	2	//0: VComm����Vref0 ; 1: Ip1����Vref0 ; 2: ���Ԥ����+Ip1΢С����Vref0
#define VREF1_METHOD	0   //0: Vref0�̶�  ��1�����

#define O2CAVGTIME		30
#define NOxCAVGTIME 	80
#define Vref0AVGTIME 	10
#define Vref1AVGTIME 	10
#define NOxAVGTIME 		10

#define NORAML_LAUNCH_ENABLE 1	//ʹ�ܳ��淽ʽ����������
#endif /* CONFIG_H_ */
