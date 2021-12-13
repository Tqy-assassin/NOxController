/*
 * common.c
 *
 *  Created on: 2019年7月11日
 *      Author: tianqingyuan
 */

#include "uart.h"
#include "rtc.h"
#include "flash.h"
#include "string.h"
#include "common.h"
#include "ics.h"
#include <string.h>
#include "printf.h"

void UART0_ISR(void);			//串口中断函数

#ifdef DEBUG
static void Parse_UartCmd(void);	//串口接受函数
#endif
#ifdef DEBUG
void printf_handle(void);			//串口打印
#endif

#ifdef DEBUG
static char uart_line_buf[256] = {0};
static uint8_t uart_rcvdata;
static uint8_t command_update = FALSE;
#endif

#ifdef DEBUG
void UART0_ISR (void)		//串口中断
{
	if(UART0->S1 & UART_S1_RDRF_MASK){
		uart_rcvdata = UART0->D;
		if(uart_rcvdata == 0x7f)
		{
			if(strlen(uart_line_buf) != 0)
				uart_line_buf[strlen(uart_line_buf)-1] = '\0';
		}
		else if(strlen(uart_line_buf) == 0)
		{
			memset(uart_line_buf, 0, sizeof(uart_line_buf));
			uart_line_buf[0] = uart_rcvdata;
		}
		else if(strlen(uart_line_buf) < sizeof(uart_line_buf) - 1)
		{
			uart_line_buf[strlen(uart_line_buf)] = uart_rcvdata;
		}

		command_update = TRUE;
	}
}
#endif





#ifdef DEBUG
static void Parse_UartCmd(void)		//串口命令解析
{
	measureCoeffi_Typedef* PCLT = CLT_pointer();
	CalculateCoeffi_Typedef* PCoeffi = Coeffi_pointer();

	int params = 0;
	float param[6] = {0};

	if(sscanf(uart_line_buf, "t %f", param) == 1){
			Vref0_V = param[0];

			printstring("\r\nVref0:"); printfloat(Vref0_V);
			printstring("(V)                    \r\n");
			MOVEUP(2);
	}else if(sscanf(uart_line_buf, "ip0 %f %f %f", param, param+1, param+2) == 3){
		Vref1.Kp = param[0];
		Vref1.Ki = param[1];
		Vref1.Kd = param[2];

		printstring("\r\nKi:"); printfloat(Vref1.Kp);
		printstring(" Kp:"); printfloat(Vref1.Ki);
		printstring(" Kd:"); printfloat(Vref1.Kd);
		printstring("                                                            \r\n");
		MOVEUP(2);
	}
	else if(sscanf(uart_line_buf, "l %f", param) == 1){
		PCLT->Vref0Limit = param[0];
	}
	else if(sscanf(uart_line_buf, "s %f %f %f", param, param+1, param+2) == 3){
			Ip0_Value = param[0];
			Ip1_Value = param[1];
			Ip2_Value = param[2];

			printstring("\r\nIp0:"); printfloat(Ip0_Value);
			printstring("(uA) Ip1:"); printfloat(Ip1_Value);
			printstring("(uA) Ip2:"); printfloat(Ip2_Value);
			printstring("(uA)                                                        \r\n");
			MOVEUP(2);
	}else if(sscanf(uart_line_buf, "p %f %f %f %f %f", param,param+1,param+2,param+3,param+4) == 5){
		PCLT->Ip1 = param[0];
		PCLT->Vref0Limit = param[1];
		PCLT->Vref1 = param[2];
		PCLT->Vref2 = param[3];
		PCLT->temper = param[4];

		CLT_store(CLT_pointer());

		printstring("\r\nIp1"); printfloat(PCLT->Ip1);
		printstring("(V) Vref0Limit:"); printfloat(PCLT->Vref0Limit);
		printstring("(V) Vref1:"); printfloat(PCLT->Vref1);
		printstring("(V) Vref2:"); printfloat(PCLT->Vref2);
		printstring("(V) Temper:"); printfloat(PCLT->temper);
		printstring("(V)                                                          \r\n");
		MOVEUP(2);
	}else if(sscanf(uart_line_buf, "p %f %f %f", param,param+1,param+2) == 3){
		PCLT->Vref1 = param[0];
		PCLT->Vref2 = param[1];
		PCLT->temper = param[2];

		CLT_store(CLT_pointer());

		printstring("\r\nVref1:"); printfloat(PCLT->Vref1);
		printstring("(V) Vref2:"); printfloat(PCLT->Vref2);
		printstring("(V) Temper:"); printfloat(PCLT->temper);
		printstring("(V)                                                          \r\n");
		MOVEUP(2);
	}else if(sscanf(uart_line_buf,"debug %d",&params) == 1){
		DEBUG_FLAG = params;
	}else if(sscanf(uart_line_buf,"stage %d",&params) == 1){
		ASIC_FLAG = params;
	}else if(sscanf(uart_line_buf,"c %f %f %f %f %f %f",param,param+1,param+2,param+3,param+4,param+5) == 6){
		PCoeffi->O2_k = param[0];
		PCoeffi->O2_d = param[1];
		PCoeffi->NOx_A1 = param[2];
		PCoeffi->NOx_B1 = param[3];
		PCoeffi->NOx_A2 = param[4];
		PCoeffi->NOx_B2 = param[5];
		PCoeffi->Save_flag = LinearEquation;
		Coeffi_store(Coeffi_pointer());

		printstring("\r\nO2_k:"); printfloat(PCoeffi->O2_k);
		printstring(" O2_d:"); printfloat(PCoeffi->O2_d);
		printstring(" NOx_A1:"); printfloat(PCoeffi->NOx_A1);
		printstring(" NOx_B1:"); printfloat(PCoeffi->NOx_B1);
		printstring(" NOx_A2:"); printfloat(PCoeffi->NOx_A2);
		printstring(" NOx_B2:"); printfloat(PCoeffi->NOx_B2);
		printstring("                                     \r\n");
		MOVEUP(2);
	}else if(strstr(uart_line_buf,"start") == uart_line_buf){
		StartTimer = Gets_Clock_value();
		stop_NOValueOffsetTimer();
		ASIC_FLAG = 1;
	}else if(strstr(uart_line_buf, "coeffi display") == uart_line_buf){
		Coeffi_retrieve(Coeffi_pointer());
		printf("coefficient:\r\nO2_k:"); printfloat(PCoeffi->O2_k);
		printf(" O2_d:"); printfloat(PCoeffi->O2_d);
		if(PCoeffi->Save_flag == QuadraticEquation){
			printf(" NOx_a1:"); printfloat(PCoeffi->NOx_a1);
			printf(" NOx_b1:"); printfloat(PCoeffi->NOx_b1);
			printf(" NOx_c1:"); printfloat(PCoeffi->NOx_c1);
			printf(" NOx_a2:"); printfloat(PCoeffi->NOx_a2);
			printf(" NOx_b2:"); printfloat(PCoeffi->NOx_b2);
			printf(" NOx_c2:"); printfloat(PCoeffi->NOx_c2);
			printf(" NOx_a3:"); printfloat(PCoeffi->NOx_a3);
			printf(" NOx_b3:"); printfloat(PCoeffi->NOx_b3);
			printf(" NOx_c3:"); printfloat(PCoeffi->NOx_c3);
		}else if(PCoeffi->Save_flag == LinearEquation){
			printf(" NOx_A1:"); printfloat(PCoeffi->NOx_A1);
			printf(" NOx_B1:"); printfloat(PCoeffi->NOx_B1);
			printf(" NOx_A2:"); printfloat(PCoeffi->NOx_A2);
			printf(" NOx_B2:"); printfloat(PCoeffi->NOx_B2);
			printf(" NOx_A3:"); printfloat(PCoeffi->NOx_A3);
			printf(" NOx_B3:"); printfloat(PCoeffi->NOx_B3);
			printf(" NOx_A4:"); printfloat(PCoeffi->NOx_A4);
			printf(" NOx_B4:"); printfloat(PCoeffi->NOx_B4);
		}else{
			printf("Error:Save_flag = 0x%X", PCoeffi->Save_flag);
		}
		printf("                                     \r\n");
	}else{
		printf("\r\nError format:%s\r\n", uart_line_buf);
		MOVEUP(1);
	}
}
#endif



void sysinit (void)
{
	/* Perform processor initialization */
	ICS_ConfigType ICS_set={0};		/* Declaration of ICS_setup structure */
	ICS_set.u8ClkMode=ICS_CLK_MODE_FEI;
	ICS_set.bdiv=0;
	ICS_Init(&ICS_set);             /*Initialization of clock at 48Mhz*/

#ifdef DEBUG
	SIM_PINSEL0 |= SIM_PINSEL_UART0PS_MASK;
	UART_ConfigType UART_Config={{0}};

	UART_Config.sctrl1settings.bits.bM=0;  	/* 8 bit mode*/
	UART_Config.sctrl1settings.bits.bPe=0;	/* No hardware parity generation or checking*/
	UART_Config.bSbns=0;					/* One stop bit*/
	UART_Config.sctrl2settings.bits.bRe=1;	/* Enable Receiver*/
	UART_Config.sctrl2settings.bits.bTe=1;	/* Enable Transmitter*/
	UART_Config.sctrl2settings.bits.bRie=1;
	UART_Config.u32SysClkHz = BUS_CLK_HZ;   	/* Bus clock in Hz*/
	UART_Config.u32Baudrate = 115200;     	/* UART baud rate */

	UART_SetCallback(UART0,UART0_ISR);
	UART_Init(UART0,&UART_Config);			/*Initialization of UART utilities*/

	RESET_CURSOR();
	CLEAR();
	printf("\rICS and UART initialization successful.\r\n");
#endif
}
/*****************************************************************************//*!
*
* @brief delay certain period of time in microseconds.
*
* @param[in]    u32TimeUS   delay value in microseconds.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void DelayUS(uint32_t u32TimeUS)
{
	uint32_t n = 0;
	uint32_t count;
	count = u32TimeUS*(BUS_CLK_HZ/1000000)/10;
	while(n < count){
		n++;
	}
//    RTC_ConfigType  RTC_Config = {0};
//
//
//    /* configure RTC to 1us period */
//    RTC_Config.u16ModuloValue = u32TimeUS/(100000000/BUS_CLK_HZ)-1;
//    RTC_Config.bInterruptEn   = FALSE;                    /*!< enable interrupt */
//    RTC_Config.bClockSource   = RTC_CLKSRC_BUS;           /*!< clock source is bus clock */
//    RTC_Config.bClockPrescaler = RTC_CLK_PRESCALER_100;    /*!< prescaler is 100 */
//
//    RTC_Init(&RTC_Config);
//    while(!RTC_GetFlags());
//    RTC_ClrFlags();
//    RTC_DeInit();
}


uint8_t Hysteresis_Comparator(int16_t input,int16_t lower,int16_t upper){
	static uint8_t Q = 0;
	if(input > upper){
		Q = 1;
	}
	if(input < lower){
		Q = 0;
	}
	return Q;
}





#ifdef DEBUG
void printstring(char* string)
{
	char* ptr = string;
	while(*ptr != '\0'){
		UART_PutChar(UART0,*(ptr));
		ptr++;
	}
}
void printfloat(float value)
{
	char string[12]={"           "};
	uint8_t i;
	char sign;
	int temp = value*10000;
	if(temp < 0){
		sign = '-';
		temp = -temp;
	}else{
		sign = ' ';
	}
	for(i = 1;i<11;i++)
	{
		if(i == 5){
			string[11-i] = '.';
			continue;
		}
		string[11-i] = temp%10 + '0';
		temp /= 10;
	}
	for(i = 1;i<5;i++){
		if(string[i] != '0'){
			break;
		}
		string[i] = ' ';
	}
	string[i-1] = sign;
	for(i = 10;i>7;i--){
		if(string[i] != '0'){
			break;
		}
		string[i] = ' ';
	}
	string[11] = '\0';
	printstring(string);
}

void printHex(unsigned int val)
{
	int i;
	unsigned int arr[8];

	/* 先取出每一位的值 */
	for (i = 0; i < 8; i++)
	{
		arr[i] = val & 0xf;
		val >>= 4;   /* arr[0] = 2, arr[1] = 1, arr[2] = 0xF */
	}

	/* 打印 */
	printstring("0x");
	for (i = 7; i >=0; i--)
	{
		if (arr[i] >= 0 && arr[i] <= 9)
			UART_PutChar(UART0,arr[i] + '0');
		else if(arr[i] >= 0xA && arr[i] <= 0xF)
			UART_PutChar(UART0,arr[i] - 0xA + 'A');
	}
}

#endif


