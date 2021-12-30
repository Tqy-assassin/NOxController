/*
 * can_manage.c
 *
 *  Created on: 2021��4��1��
 *      Author: sunkaixiang
 */

#include "can_manage.h"
#ifdef CAN_CONTROL
#include "vendor.h"
#include "CAN.h"
#include "can_api.h"
#include "clock.h"
#include "ftm.h"
#include "flash_manage.h"
#include <string.h>
#include "device_state_manger.h"
#include "heating_controller.h"
#include "power_manage.h"
#include "sensor_control.h"
#include "Filter.h"


#define O2_k_STEP		(3E-10)
#define O2_k_OFFSET		(2147483647)
#define O2_d_STEP		(3E-9)
#define O2_d_OFFSET		(2147483647)
#define NOx_A1_STEP		(3E-8)
#define NOx_A1_OFFSET	(2147483647)
#define NOx_B1_STEP		(3E-6)
#define NOx_B1_OFFSET	(2147483647)
#define NOx_A2_STEP		(3E-7)
#define NOx_A2_OFFSET	(2147483647)
#define NOx_B2_STEP		(3E-6)
#define NOx_B2_OFFSET	(2147483647)

#define Vref1_STEP		(1E-3)
#define Vref1_OFFSET	0
#define Vref2_STEP		(1E-3)
#define Vref2_OFFSET	0
#define Temp_STEP		(1E-3)
#define Temp_OFFSET		0
#define Vref0_STEP		(1E-3)
#define Vref0_OFFSET	0
#define Ip1_STEP		(1E-3)
#define Ip1_OFFSET		0

#define IP0_STEP		(5E-6)
#define IP0_OFFSET		(-10000)
#define IP2_STEP		(1E-8)
#define IP2_OFFSET		(-20)

enum PrivateCmdList{
	Cmd_Request = 1,
	Cmd_Reply = 2,
	Cmd_Ask = 3,
	Cmd_Control = 4,
	Cmd_Connect,
	Cmd_Restart,
	Cmd_Start,
	Cmd_Stop,
};

enum PrivateParamList{
	Param_Version = 1,
	Param_Volage,
	Param_Vref0,
	Param_Vref1,
	Param_Vref2,
	Param_Ip0,
	Param_Ip1,
	Param_Ip2,
	Param_Comm,
	Param_Temp,
	Param_Test,
	Param_PWMDuty,
	Param_Stage,
	Param_O2C,
	Param_NOC,
	Param_Inspect,

	Coeffi_Save,
  	Coeffi_O2_k,
  	Coeffi_O2_d,
	//A1,B1,A2,B2���㵪��ֵ���㷽��
  	Coeffi_NOx_A1,
  	Coeffi_NOx_B1,
  	Coeffi_NOx_A2,
  	Coeffi_NOx_B2,
	CLT_Save,
  	CLT_Vref1,
  	CLT_Vref2,
  	CLT_Temper,
 	CLT_Vref0Limit,
 	CLT_Ip1,

	Calibrate_Start,
	Calibrate_Stop,

	Param_Status,
//	Param_ID

	PID_IP0_P,
	PID_IP0_I,
	PID_IP0_D,
	PID_IP1_P,
	PID_IP1_I,
	PID_IP1_D,
	PID_IP2_P,
	PID_IP2_I,
	PID_IP2_D,
//A1,B1,A2,B2,A3,B3,A4,B4���㵪��ֵ���㷽��
  	Coeffi_NOx_A3,
  	Coeffi_NOx_B3,
  	Coeffi_NOx_A4,
  	Coeffi_NOx_B4,

	//���㵪��ֵ���η���
  	Coeffi_NOx_a1,
  	Coeffi_NOx_b1,
  	Coeffi_NOx_c1,
  	Coeffi_NOx_a2,
  	Coeffi_NOx_b2,
  	Coeffi_NOx_c2,
	Coeffi_NOx_a3,
  	Coeffi_NOx_b3,
  	Coeffi_NOx_c3,

	TransmitSpeed,
	RunTime,

    Coeffi_NOx_sb1,
    Coeffi_NOx_sc1,
    Coeffi_NOx_sb2,
    Coeffi_NOx_sc2,
    Coeffi_NOx_sb3,
    Coeffi_NOx_sc3,

    Coeffi_O2_Pressure,
    Coeffi_NOx_Pressure,
	Param_Manufacturers
};

enum PrivateTypeList{
	Type_Uint32 = 1,
	Type_Float,
	Type_Uint8
};

typedef struct{
	union{
		struct{
			uint8_t cmd;
			uint8_t param;
			uint8_t type;
			uint8_t res;
			union{
				uint32_t Uint32;
				float Float;
				uint8_t Uint8[4];
			}datas;
		}Paivate_Cmd;
		uint8_t RxData[8];
	};
}CANPaivateFrameType;

extern LoopArray Ip0Totle;
extern LoopArray Ip2Totle;

#ifdef CAN_CONTROL
void FTM1_init(void);
void FTM1_Task(void);
#endif

extern float Kp[][5];
extern float Ki[][5];
extern float Kd[][5];
extern uint32_t StartTimer;
extern uint32_t Start_Timer;
extern uint32_t SourceAddr;
extern uint8_t InspectResult[];

extern uint8_t NoPIDflag;
extern LoopArray NOxTotle;
extern float Vref;
uint32_t NOValueOffsetTimer = 0;	//�������ֵ������ʱ��
uint32_t O2CValueOffsetTimer = 0;
//uint8_t Version[] = VERSION;
uint8_t Version[] = VERSION;
uint8_t Manufacturers[] = VERSION;

uint32_t DATA2_1STimer = 0;

float O2C = 4;		//O2Ũ�� %
float NOxC;		//NOxŨ�� ppm
float NOxOUT;		//NOxŨ�� ppm
void Cmd_StopHandle(void);
void Can_Init(void);			//����CAN������ʼ��
void CAN_RxHandle(void);			//CAN�����ж�
void AtmosphereCalculate(void);		//���ռ���
void Status_Detect(void);			//״̬�ж�
void CAN_TxTask(void);				//CAN���ݴ���������
void CAN_TxDataTask(void);			//CAN������ֵ
void CAN_CMDTransmit(uint16_t cmdid,uint8_t* data,uint32_t Timeout_ms);			//CAN��������
uint8_t CAN_Start = 0;			//¶��������ʶ
uint32_t CAN_StartTimemr = 0;	//¶��������ʱ��
uint32_t CAN_StopTimemr = 0;	//¶��ֹͣʱ��
uint8_t CAN_Stop = 0;			//¶��ֹͣ��ʶ
uint16_t PaivateTxInterval = 50;
uint32_t PeriodPaivateTxTimer = 0;

uint32_t CAN_RxManyPostbackTimemr = 0;

uint32_t ConnectTimer = 0;
uint8_t ConnectUSBtoCAN = 0;
//CANRxFrameDataType CANTxFrame;
float Torque;
float Speed;
float SpeedAdjust;
float TorqueLosses;
float GasMassFlow;

//#define Alpha_NOpres (0.135)
//#define Lambda	(1)
//#define KNO2 	(0.825)
//#define KNH3	(1.05)
//float Pactual = 101.3;	//��ѹ
//float NH3C = 0;	//NH3Ũ��

/*#include "ASIC_controller.h"
#define COMM			(-2)

void test(void){
	//uint8_t h=0x3f;
	//uint8_t l=0x14;
	uint8_t h=0x05;
	uint8_t l=0xf5;
	int16_t data1 = 0;
	float data2 = 0, data3 = 0, data4 = 0;
	data1 = AnalyseDatas(h,l);//0xff14
	data2 = (float)data1;//-236
	data3 = data2*COMM/8192;//0.0576171875
	data4 = (float)data1*COMM/8192;
	while(1){
		ItemInfoType sTxFrame;
		CANPaivateFrameType TxFrame;
		extern uint32_t SourceAddr;
		sTxFrame.ID_Type.ID = PDU_P(4) | PDU_R(0) | PDU_DP(0) | PDU_PGN(PrivateCmd_Intake) | PDU_SA(SourceAddr);
		sTxFrame.bIsExtOrStand = 1;
		sTxFrame.bIsRemoteFrame = 0;
		sTxFrame.u32DataLength = 8;
		sTxFrame.u8BPR = 0x10;

		TxFrame.Paivate_Cmd.cmd = Cmd_Ask;
		TxFrame.Paivate_Cmd.type = Type_Float;

		TxFrame.Paivate_Cmd.param = Param_Comm;
		TxFrame.Paivate_Cmd.datas.Float = data4;
		memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
		CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);
	}
}
*/

void delchar(uint8_t *p,int k,int n)
{
    int nn,nnn;
    uint8_t *q;
    k--;
    nn=0;
    q=p;
    while (*q){
    	q++;
    	nn++;
    }
    nnn=nn-k-n; if ( nnn<0 ) return;
    p+=k;
    q=p;
    q+=n;
    while(*q){
    	(*p)=(*q);
    	p++; q++;
    }
    (*p)=0;
}


void stop_NOValueOffsetTimer(){
	NOValueOffsetTimer = 0;
}

void start_NOValueOffsetTimer(uint32_t value){
	NOValueOffsetTimer = value;
}

void start_O2CValueOffsetTimer(uint32_t value){
	O2CValueOffsetTimer = value;
}

float get_O2C(void){
	return O2C;
}

float get_NOxC(void){
	return NOxC;
}

static MSCAN_ConfigType sMSCANConfig = {{0}};
void CAN_PC_Init(void)
{
	MSCAN_GlobeVaribleInit(MSCAN);
	SIM->PINSEL1 |= SIM_PINSEL1_MSCANPS_MASK;
//Baud_RATE = 24M/(BAUD_RATE_BRP+1)/(BAUD_RATE_SJW)/(1 + BAUD_RATE_TSEG1+BAUD_RATE_TSEG2)
#ifdef CAN_BAUDRATE_250kbps
	sMSCANConfig.sBaudRateSetting.SJW = SJW_1TQ;
	sMSCANConfig.sBaudRateSetting.BRP = 3;
	sMSCANConfig.sBaudRateSetting.SAMP = 1;
	sMSCANConfig.sBaudRateSetting.TSEG1= TSEG_16;
	sMSCANConfig.sBaudRateSetting.TSEG2= TSEG_7;
#elif defined(CAN_BAUDRATE_500kbps)
	sMSCANConfig.sBaudRateSetting.SJW = SJW_1TQ;
	sMSCANConfig.sBaudRateSetting.BRP = 3;
	sMSCANConfig.sBaudRateSetting.SAMP = 1;
	sMSCANConfig.sBaudRateSetting.TSEG1= TSEG_8;
	sMSCANConfig.sBaudRateSetting.TSEG2= TSEG_3;
#endif
	sMSCANConfig.u8IDARMode = ID_ACCEPT_MODE_TWO32;
	sMSCANConfig.u32IDAR0 = IDAR0;
	sMSCANConfig.u32IDMR0 = IDMR0;
	sMSCANConfig.u32IDAR1 = IDAR1;
	sMSCANConfig.u32IDMR1 = IDMR1;

	sMSCANConfig.sSetting.bCanEn = 1;
	sMSCANConfig.sSetting.bCLKSRC = 1;
//	sMSCANConfig.sSetting.bLoopModeEn = 1;
	sMSCANConfig.sSetting.bRxFullIEn=1;
	sMSCANConfig.sSetting.bTimerEn=0;
	sMSCANConfig.sSetting.bOverRunIEn=0;
	sMSCANConfig.sSetting.bStatusChangeIEn=0;
	sMSCANConfig.sSetting.bTxEmptyIEn=1;

//	MSCAN_SetRxCallBack(CAN_RxTask);

	CAN_Init(MSCAN,&sMSCANConfig);
#ifdef DEBUG
	printf("\r\nCAN bus initialization successful.\r\n");
#endif
	delchar(Version,1,4);
	delchar(Version,3,1);
	delchar(Manufacturers,4,6);

}
void CAN_IfConnectUSBtoCAN(void){
	if(ConnectUSBtoCAN){
		CAN_StartTimemr = Gets_Clock_value();
		if(clock_time_exceed(ConnectTimer,5000)){
			ConnectUSBtoCAN = 0;
		}
	}
}

void CAN_OtherHandle(void){
	if((CAN_Stop == 1) && clock_time_exceed(CAN_StopTimemr,5000)){
		CAN_Stop = 0;
		CAN_StopTimemr = 0;
		Cmd_StopHandle();
	}
}

void can_receive(void){
#ifdef CAN_CONTROL
	CAN_IfConnectUSBtoCAN();
	CAN_RxHandle();
	CAN_OtherHandle();
#endif
}

void can_transmit(void){
	if(clock_time_exceed(PeriodPaivateTxTimer, PaivateTxInterval)){
		PeriodPaivateTxTimer = Gets_Clock_value();
#ifdef CAN_CONTROL
	if(ConnectUSBtoCAN){
		Status_Detect();
		AtmosphereCalculate();
		CAN_TxDataTask();
	}
#endif
#ifdef UART_CONTROL
			printf_handle();		//print

			if(strlen(uart_line_buf))
			MOVERIGHT(strlen(uart_line_buf));
#endif
  }
}

float Twopoint_equation(float x1,float y1,float x2,float y2,float x){
	return ((x - x2)/(x1 - x2))*(y1 - y2) + y2;
}

float ZeroFitting(float NOx){
	return 0.00067*NOx*NOx*NOx-0.018*NOx*NOx+0.156*NOx+0.001;
}





uint8_t RS_flipflop(uint8_t R,uint8_t S){
    static uint8_t Q = 0;
    if(R == 0 && S == 0){

    }else if(R == 0 && S == 1){
        Q = 1;
    }else if(R == 1 && S == 0){
        Q = 0;
    }else{

    }
    return Q;
}

//1:Ԥ��  0:����
//uint8_t O2CNOx_To_RS(float O2C,float NOx){
//	return RS_flipflop((uint8_t)(NOx > 50), (uint8_t)(O2C >= 20.5));
//}

float NOxHandle(float O2C,float NOxCAN){
	static float PRNOxCAN;
	if(RS_flipflop((uint8_t)(NOxCAN > 50), (uint8_t)(O2C >= 20.5))){//Ԥ��ģ��
		if(clock_time_exceed(O2CValueOffsetTimer,500) && RS_flipflop((uint8_t)(NOxCAN > 50), (uint8_t)(O2C >= 20.5))){
			O2CValueOffsetTimer = Gets_Clock_value();
			PRNOxCAN = 0.95 * ZeroFitting(NOxCAN) + 0.05 * NOxCAN;
		}
	}else{//����ģ��
		O2CValueOffsetTimer = Gets_Clock_value();
		PRNOxCAN = NOxCAN;
	}
	return PRNOxCAN;
}

void AtmosphereCalculate(void){
	float Ip0Avg,Ip2Avg;
	float NOxCAN,PRNOxCAN;

	CalculateCoeffi_Typedef* PCoe = Coeffi_pointer();
#if Filter_type == AvgFilter
	Ip0Avg = getavg(&Ip0Totle);
	Ip2Avg = getavg(&Ip2Totle);
#elif Filter_type == LowFilter
	Ip0Avg = filter_get(&Ip0Filter);
	Ip2Avg = filter_get(&Ip2Filter);
#elif Filter_type == DF2Filter
	Ip0Avg = filter_get(&Ip0Filter);
	Ip2Avg = filter_get(&Ip2Filter);
#else
	Ip0Avg = Ip_Value.Ip0_Value;
	Ip2Avg = Ip_Value.Ip2_Value;
#endif
	Ip2Avg = (float)((int32_t)(Ip2Avg*1000))/1000;
	O2C = (Ip0Avg)*PCoe->O2_k + PCoe->O2_d;
	if(O2C < 2){//O2<2%
		Status_NOxnValid();
	}
	if(PCoe->Save_flag == QuadraticEquation){
		NOxCAN = (PCoe->NOx_a1*O2C*O2C + PCoe->NOx_b1*O2C + PCoe->NOx_c1)*Ip2Avg*Ip2Avg + (PCoe->NOx_a2*O2C*O2C + PCoe->NOx_b2*O2C + PCoe->NOx_c2)*Ip2Avg + (PCoe->NOx_a3*O2C*O2C + PCoe->NOx_b3*O2C + PCoe->NOx_c3);
	}else if(PCoe->Save_flag == LinearEquation){
		NOxCAN = (PCoe->NOx_A1*O2C + PCoe->NOx_B1)*((float)((int32_t)(Ip2Avg*1000))/1000) + (PCoe->NOx_A2*O2C + PCoe->NOx_B2);
		if(NOxCAN > 600){
			NOxCAN = (PCoe->NOx_A3*O2C + PCoe->NOx_B3)*((float)((int32_t)(Ip2Avg*1000))/1000) + (PCoe->NOx_A4*O2C + PCoe->NOx_B4);
		}else if(NOxCAN > 400){
			float rate;
			float NOxC_1, NOxC_2;
			NOxC_1 = NOxCAN;
			NOxC_2 = (PCoe->NOx_A3*O2C + PCoe->NOx_B3)*((float)((int32_t)(Ip2Avg*1000))/1000) + (PCoe->NOx_A4*O2C + PCoe->NOx_B4);
			rate = (600 - NOxCAN) / (600 - 400);
			NOxCAN = NOxC_1 * rate + NOxC_2 * (1-rate);
		}
	}else if(PCoe->Save_flag == Quadratic_LinearEquation){
		float sA = PCoe->NOx_sb1*O2C + PCoe->NOx_sc1;
		float sB = PCoe->NOx_sb2*O2C + PCoe->NOx_sc2;
		float sC = PCoe->NOx_sb3*O2C + PCoe->NOx_sc3;
		NOxCAN = sA*Ip2Avg*Ip2Avg + sB*Ip2Avg + sC;
	}

	O2C = O2C > 21.5 ? 21.5 : (O2C < -12 ? -12 : O2C);

	if(NOxCAN < 0){
		NOxCAN = 1 / (2 - NOxCAN);
	}

	PRNOxCAN = NOxHandle(O2C,NOxCAN);

	NOxC = PRNOxCAN + Look_Aging_factor(get_new_run_time())*(Lookup_AGEtable(PRNOxCAN,O2C));

	push(&NOxTotle,NOxC);
	NOxOUT = getavg(&NOxTotle);
	//NOxC = NOxC > 3076 ? 3076 : (NOxC < -200 ? -200 : NOxC);
	NOxOUT = NOxOUT > 3076 ? 3076 : (NOxOUT < 0 ? 0 : NOxOUT);

}

uint8_t CAN_TransmitItem(MSCAN_Type *pCANx,ItemInfoPtr pTxItemInfo,FrameBufferInfoPtr pTxBuffInfo)
{
	static uint8_t Failed_count = 0;
	if(CAN_TransmitItemByInt(pCANx, pTxItemInfo, pTxBuffInfo) == FALSE){
		Failed_count++;
		if(Failed_count >= 100){
			CAN_DeInit(pCANx);
			CAN_Init(pCANx, &sMSCANConfig);
			Failed_count = 0;
		}
		return FALSE;
	}
	return TRUE;
}

void CAN_CMDTransmit(uint16_t cmdid, uint8_t* data, uint32_t Timeout_ms)
{
	ItemInfoType sTxFrameInfo;
	uint32_t timer;
	uint8_t Status;
	sTxFrameInfo.ID_Type.ID = PDU_P(4) | PDU_R(0) | PDU_DP(0) | PDU_PGN(cmdid) | PDU_SA(SourceAddr);
	sTxFrameInfo.bIsExtOrStand = 1;
	sTxFrameInfo.bIsRemoteFrame = 0;
	sTxFrameInfo.u32DataLength = 8;
	sTxFrameInfo.u8BPR = 0x10;
	memcpy(sTxFrameInfo.u8DataBuff, data, 8);
	timer = Gets_Clock_value();
	do{
		Status = CAN_TransmitItem(MSCAN,&sTxFrameInfo,&sCAN_TxBuff);
		if(clock_time_exceed(timer,Timeout_ms)){
			break;
		}
	}while(!Status);
}



void Status_Detect(void)
{
	IpValue * Ip = get_Ip_Value();
	ADtfValue * ADtf = get_ADtf_Value();
	if(get_working_stage() == STAGE_ENVIRONMENT){
		if((Ip->Ip0_Value < 5000) && (Ip->Ip0_Value > 100) && (ADtf->Vref1Value >= 0.35) && (ADtf->Vref1Value <= 0.5) && (NoPIDflag == 0)){
			Status_O2Valid();
		}else{
			Status_O2nValid();
		}

		if((Ip->Ip2_Value < 12) && (Ip->Ip2_Value > -12) && (ADtf->Vref2Value >= 0.4) && (ADtf->Vref2Value <= 0.6) && (NoPIDflag == 0)){
			Status_NOxValid();
		}else{
			Status_NOxnValid();
		}
	}else{
		Status_O2nValid();
		Status_NOxnValid();
	}
}


void CAN_RxHandle(void)
{
	measureCoeffi_Typedef* PCLT = CLT_pointer();
	CalculateCoeffi_Typedef* PCoeffi = Coeffi_pointer();
	MSCAN_FrameType pRxFrame;
	CANPaivateFrameType RxFrame;
	CANPaivateFrameType TxFrame;
	uint32_t PGN;
#ifdef JUDGE_RECEIVE
	uint8_t SA;		//Source Address
	uint8_t TA;		//Target Address
#endif
	if(CAN_ReadOneFramefromBufferQueue(&pRxFrame)){
		if(pRxFrame.bIsExtOrStand){
			PGN = (pRxFrame.ID_Type.ID & PDU_PGN_MASK) >> PDU_PGN_SHIFT;
#ifdef JUDGE_RECEIVE
			SA = (uint8_t)((pRxFrame.CAN_ID.ID & PDU_SA_MASK) >> PDU_SA_SHIFT);
			if(PGN < 0xF000){
				TA = (uint8_t)PGN;
			}else{
				TA = 0xFF;
			}
#endif
		}else{
			PGN = pRxFrame.ID_Type.ID;
		}
		memcpy(RxFrame.RxData,pRxFrame.DSR,8);
		switch(PGN){
		case PrivateCmd_Intake:
		case PrivateCmd_Outlet:
			switch(RxFrame.Paivate_Cmd.cmd)
			{
			case Cmd_Connect:
				ConnectUSBtoCAN = RxFrame.Paivate_Cmd.param;
				ConnectTimer = Gets_Clock_value();
				TxFrame.Paivate_Cmd.cmd = Cmd_Connect;
				TxFrame.Paivate_Cmd.param = ConnectUSBtoCAN;
				CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
				break;
			case Cmd_Request:
				switch(RxFrame.Paivate_Cmd.param){
				case Param_Version:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Param_Version;
					TxFrame.Paivate_Cmd.type = Type_Uint8;
					memcpy(TxFrame.Paivate_Cmd.datas.Uint8,Version,4);
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Param_Manufacturers:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Param_Manufacturers;
					TxFrame.Paivate_Cmd.type = Type_Uint8;
					memcpy(TxFrame.Paivate_Cmd.datas.Uint8,Manufacturers,4);
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Param_Volage:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Param_Volage;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = get_WorkVoltage();
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case TransmitSpeed:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = TransmitSpeed;
					TxFrame.Paivate_Cmd.type = Type_Uint32;
					TxFrame.Paivate_Cmd.datas.Uint32 = (int)PaivateTxInterval;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case RunTime:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = RunTime;
					TxFrame.Paivate_Cmd.type = Type_Uint32;
					TxFrame.Paivate_Cmd.datas.Uint32 = (get_new_run_time()) / 60;//min
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_O2_k:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_O2_k;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->O2_k;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_O2_d:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_O2_d;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->O2_d;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sb1:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sb1;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sb1;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sc1:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sc1;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sc1;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sb2:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sb2;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sb2;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sc2:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sc2;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sc2;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sb3:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sb3;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sb3;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				case Coeffi_NOx_sc3:
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = Coeffi_NOx_sc3;
					TxFrame.Paivate_Cmd.type = Type_Float;
					TxFrame.Paivate_Cmd.datas.Float = PCoeffi->NOx_sc3;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
					break;
				default:
					break;
				}
				break;
			case Cmd_Control:
				switch(RxFrame.Paivate_Cmd.param){
				case Coeffi_Save:
					ready_store_Coeffi();
//					Coeffi_store(&Coeffi);
//					Coeffi_retrieve(&Coeffi);
//					CANTxFrame.Paivate_Cmd.cmd = Cmd_Reply;
//					CANTxFrame.Paivate_Cmd.param = Coeffi_O2_k;
//					CANTxFrame.Paivate_Cmd.type = Type_Float;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.O2_k;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_O2_d;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.O2_d;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_A1;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_A1;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_B1;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_B1;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_A2;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_A2;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_B2;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_B2;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_A3;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_A3;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_B3;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_B3;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_A4;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_A4;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_NOx_B4;
//					CANTxFrame.Paivate_Cmd.datas.Float = Coeffi.NOx_B4;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = Coeffi_Save;
//					DelayUS(10000);
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
					break;
				case Coeffi_O2_k:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->O2_k = RxFrame.Paivate_Cmd.datas.Float;
						//Coeffi.O2_k = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->O2_k = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - O2_k_OFFSET)*O2_k_STEP;
						//Coeffi.O2_k = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - O2_k_OFFSET)*O2_k_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_O2_d:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->O2_d = RxFrame.Paivate_Cmd.datas.Float;
						//Coeffi.O2_d = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->O2_d = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - O2_d_OFFSET)*O2_d_STEP;
						//Coeffi.O2_d = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - O2_d_OFFSET)*O2_d_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_A1:
					PCoeffi->Save_flag = LinearEquation;
					//Coeffi.Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_A1 = RxFrame.Paivate_Cmd.datas.Float;
						//Coeffi.NOx_A1 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_A1 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A1_OFFSET) * NOx_A1_STEP;
						//Coeffi.NOx_A1 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A1_OFFSET) * NOx_A1_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_B1:
					PCoeffi->Save_flag = LinearEquation;
					//Coeffi.Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_B1 = RxFrame.Paivate_Cmd.datas.Float;
						//Coeffi.NOx_B1 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_B1 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_B1_OFFSET) * NOx_B1_STEP;
						//Coeffi.NOx_B1 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_B1_OFFSET) * NOx_B1_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_A2:
					PCoeffi->Save_flag = LinearEquation;
					//Coeffi.Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_A2 = RxFrame.Paivate_Cmd.datas.Float;
						//Coeffi.NOx_A2 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_A2 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A2_OFFSET)*NOx_A2_STEP;
						//Coeffi.NOx_A2 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A2_OFFSET)*NOx_A2_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_B2:
					PCoeffi->Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_B2 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_B2 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_B2_OFFSET) * NOx_B2_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_A3:
					PCoeffi->Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_A3 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_A3 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A1_OFFSET) * NOx_A1_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_B3:
					PCoeffi->Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_B3 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_B3 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_B1_OFFSET) * NOx_B1_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_A4:
					PCoeffi->Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_A4 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_A4 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_A2_OFFSET)*NOx_A2_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_B4:
					PCoeffi->Save_flag = LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_B4 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCoeffi->NOx_B4 = (int32_t)(RxFrame.Paivate_Cmd.datas.Uint32 - NOx_B2_OFFSET) * NOx_B2_STEP;
					}
					ready_store_Coeffi();
					break;
				case Coeffi_NOx_a1:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_a1 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_b1:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_b1 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_c1:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_c1 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_a2:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_a2 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_b2:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_b2 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_c2:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_c2 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_a3:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_a3 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_b3:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_b3 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_c3:
					PCoeffi->Save_flag = QuadraticEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_c3 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sb1:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sb1 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sc1:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sc1 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sb2:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sb2 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sc2:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sc2 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sb3:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sb3 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_sc3:
					PCoeffi->Save_flag = Quadratic_LinearEquation;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_sc3 = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_O2_Pressure:
					PCoeffi->Save_flag = PressureCorrection;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->O2_Pressure = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case Coeffi_NOx_Pressure:
					PCoeffi->Save_flag = PressureCorrection;
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCoeffi->NOx_Pressure = RxFrame.Paivate_Cmd.datas.Float;
					}
					ready_store_Coeffi();
				break;
				case CLT_Save:
					ready_store_CLT();
//					CLT.Save_flag = 0xFA;
//					CLT_store(&CLT);
//					memcpy((uint8_t*)(&CLT),(uint8_t*)(CLT_SECTOR*FLASH_SECTOR_SIZE),sizeof(measureCoeffi_Typedef));
//					CANTxFrame.Paivate_Cmd.cmd = Cmd_Reply;
//					CANTxFrame.Paivate_Cmd.param = CLT_Ip1;
//					CANTxFrame.Paivate_Cmd.type = Type_Float;
//					CANTxFrame.Paivate_Cmd.datas.Float = CLT.Ip1;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = CLT_Vref0Limit;
//					CANTxFrame.Paivate_Cmd.datas.Float = CLT.Vref0Limit;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = CLT_Vref1;
//					CANTxFrame.Paivate_Cmd.datas.Float = CLT.Vref1;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = CLT_Vref2;
//					CANTxFrame.Paivate_Cmd.datas.Float = CLT.Vref2;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = CLT_Temper;
//					CANTxFrame.Paivate_Cmd.datas.Float = CLT.temper;
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
//					CANTxFrame.Paivate_Cmd.param = CLT_Save;
//					DelayUS(10000);
//					CAN_CMDTransmit(PrivateCmd_Intake,CANTxFrame.RxData,500);
					break;
				case CLT_Vref1:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCLT->Vref1 = RxFrame.Paivate_Cmd.datas.Float;
						//CLT_pointer()->Vref1 = RxFrame.Paivate_Cmd.datas.Float;
						//CLT.Vref1 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCLT->Vref1 = RxFrame.Paivate_Cmd.datas.Uint32 * Vref1_STEP - Vref1_OFFSET;
						//CLT.Vref1 = RxFrame.Paivate_Cmd.datas.Uint32 * Vref1_STEP - Vref1_OFFSET;
					}
					ready_store_CLT();
					break;
				case CLT_Vref2:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCLT->Vref2 = RxFrame.Paivate_Cmd.datas.Float;
						//CLT.Vref2 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCLT->Vref2 = RxFrame.Paivate_Cmd.datas.Uint32 * Vref2_STEP - Vref2_OFFSET;
						//CLT.Vref2 = RxFrame.Paivate_Cmd.datas.Uint32 * Vref2_STEP - Vref2_OFFSET;
					}
					ready_store_CLT();
					break;
				case CLT_Temper:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCLT->temper = RxFrame.Paivate_Cmd.datas.Float;
						//CLT.temper = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCLT->temper = RxFrame.Paivate_Cmd.datas.Uint32 * Temp_STEP - Temp_OFFSET;
						//CLT.temper = RxFrame.Paivate_Cmd.datas.Uint32 * Temp_STEP - Temp_OFFSET;
					}
					ready_store_CLT();
					break;
				case CLT_Vref0Limit:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCLT->Vref0Limit = RxFrame.Paivate_Cmd.datas.Float;
						//CLT.Vref0Limit = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCLT->Vref0Limit = RxFrame.Paivate_Cmd.datas.Uint32 * Vref0_STEP - Vref0_OFFSET;
						//CLT.Vref0Limit = RxFrame.Paivate_Cmd.datas.Uint32 * Vref0_STEP - Vref0_OFFSET;
					}
					ready_store_CLT();
					break;
				case CLT_Ip1:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						PCLT->Ip1 = RxFrame.Paivate_Cmd.datas.Float;
						//CLT.Ip1 = RxFrame.Paivate_Cmd.datas.Float;
					}else if(RxFrame.Paivate_Cmd.type == Type_Uint32){
						PCLT->Ip1 = RxFrame.Paivate_Cmd.datas.Uint32 * Ip1_STEP - Ip1_OFFSET;
						//CLT.Ip1 = RxFrame.Paivate_Cmd.datas.Uint32 * Ip1_STEP - Ip1_OFFSET;
					}
					ready_store_CLT();
					break;
				case PID_IP0_P:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kp[0][0] = Kp[0][1] = Kp[0][2] = Kp[0][3] = Kp[0][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP0_I:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Ki[0][0] = Ki[0][1] = Ki[0][2] = Ki[0][3] = Ki[0][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP0_D:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kd[0][0] = Kd[0][1] = Kd[0][2] = Kd[0][3] = Kd[0][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP1_P:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kp[1][0] = Kp[1][1] = Kp[1][2] = Kp[1][3] = Kp[1][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP1_I:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Ki[1][0] = Ki[1][1] = Ki[1][2] = Ki[1][3] = Ki[1][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP1_D:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kd[1][0] = Kd[1][1] = Kd[1][2] = Kd[1][3] = Kd[1][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP2_P:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kp[2][0] = Kp[2][1] = Kp[2][2] = Kp[2][3] = Kp[2][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP2_I:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Ki[2][0] = Ki[2][1] = Ki[2][2] = Ki[2][3] = Ki[2][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case PID_IP2_D:
					if(RxFrame.Paivate_Cmd.type == Type_Float){
						Kd[2][0] = Kd[2][1] = Kd[2][2] = Kd[2][3] = Kd[2][4] = RxFrame.Paivate_Cmd.datas.Float;
					}
					break;
				case TransmitSpeed:
					PaivateTxInterval = RxFrame.Paivate_Cmd.datas.Uint32;
					TxFrame.Paivate_Cmd.cmd = Cmd_Reply;
					TxFrame.Paivate_Cmd.param = TransmitSpeed;
					TxFrame.Paivate_Cmd.type = Type_Uint32;
					TxFrame.Paivate_Cmd.datas.Uint32 = (int)PaivateTxInterval;
					CAN_CMDTransmit(PrivateCmd_Intake,TxFrame.RxData,500);
				default:
					break;
				}
				break;
			case Cmd_Restart:
				__DSB();
				SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEYSTAT_Pos)|SCB_AIRCR_SYSRESETREQ_Msk);
				__DSB();
				while(1);
			case Cmd_Start:
				//CAN_Stop = 0;
				//CAN_StopTimemr = 0;
				CAN_StartTimemr = Gets_Clock_value();
				CAN_Start = 1;
				if(get_working_stage() == STAGE_IDLE){
					NoPIDflag = 1;
					set_working_stage(STAGE_PREHEATING);
					StartTimer = Gets_Clock_value();
					Start_Timer = Gets_Clock_value();
					stop_NOValueOffsetTimer();
				}
				break;
			case Cmd_Stop:
				//CAN_StopTimemr = Gets_Clock_value();
				//CAN_Stop = 1;
				Cmd_StopHandle();
				break;
			default:
				break;
			}
		default:
			CAN_RxTask(PGN, (CANRxFrameDataType*)&RxFrame);
			break;
		}
	}
}

void Cmd_StopHandle(void){
	CAN_Start = 0;
	CAN_StartTimemr = 0;
	set_working_stage(STAGE_IDLE);
}


void CAN_TxDataTask(void){
	ItemInfoType sTxFrame;
	CANPaivateFrameType TxFrame;
	IpValue * Ip = get_Ip_Value();
	ADtfValue * ADtf_Value = get_ADtf_Value();
	extern uint32_t SourceAddr;
	sTxFrame.ID_Type.ID = PDU_P(4) | PDU_R(0) | PDU_DP(0) | PDU_PGN(PrivateCmd_Intake) | PDU_SA(SourceAddr);
	sTxFrame.bIsExtOrStand = 1;
	sTxFrame.bIsRemoteFrame = 0;
	sTxFrame.u32DataLength = 8;
	sTxFrame.u8BPR = 0x10;

	TxFrame.Paivate_Cmd.cmd = Cmd_Ask;
	TxFrame.Paivate_Cmd.type = Type_Float;

	TxFrame.Paivate_Cmd.param = Param_Vref0;
	TxFrame.Paivate_Cmd.datas.Float = ADtf_Value->Vref0Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Vref1;
	TxFrame.Paivate_Cmd.datas.Float = ADtf_Value->Vref1Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Vref2;
	TxFrame.Paivate_Cmd.datas.Float = ADtf_Value->Vref2Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Ip0;
	TxFrame.Paivate_Cmd.datas.Float = Ip->Ip0_Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Ip1;
	TxFrame.Paivate_Cmd.datas.Float = Ip->Ip1_Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Ip2;
	TxFrame.Paivate_Cmd.datas.Float = Ip->Ip2_Value;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Comm;
	TxFrame.Paivate_Cmd.datas.Float = ADtf_Value->VCommValue;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Temp;
	TxFrame.Paivate_Cmd.datas.Float = ADtf_Value->VTempValue;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Test;
	TxFrame.Paivate_Cmd.datas.Float = Vref;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_PWMDuty;
	TxFrame.Paivate_Cmd.datas.Float = get_PWM_Duty_Cycle();
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Stage;
	TxFrame.Paivate_Cmd.datas.Uint32 = get_working_stage();
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_O2C;
	TxFrame.Paivate_Cmd.datas.Float = O2C;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_NOC;
	TxFrame.Paivate_Cmd.datas.Float = NOxC;
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Inspect;
	memcpy(TxFrame.Paivate_Cmd.datas.Uint8,	&InspectResult[1], 4);
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);

	TxFrame.Paivate_Cmd.param = Param_Status;
	TxFrame.Paivate_Cmd.datas.Uint32 = Status_Get();
	memcpy(sTxFrame.u8DataBuff, TxFrame.RxData, 8);
	CAN_TransmitItem(MSCAN,&sTxFrame,&sCAN_TxBuff);
}

void CAN_control(void){
	can_receive();
	can_transmit();
	if(clock_time_exceed(CAN_RxManyPostbackTimemr,20)){//20ms��һ��
		CAN_RxManyPostbackTimemr = Gets_Clock_value();
		CAN_RxManyPostback();
	}
}

void FTM1_Task(void)
{
	if(FTM_GetOverFlowFlag(FTM1)){
		FTM_ClrOverFlowFlag(FTM1);
		if(CAN_Start && CAN_StartTimemr && clock_time_exceed(CAN_StartTimemr,300*1000)){	//��ʱ��ֹͣ����
			CAN_Start = 0;
			CAN_StartTimemr = 0;
			set_working_stage(STAGE_IDLE);
		}
		Status_Detect();
		AtmosphereCalculate();
		CAN_TxTask();//DATA1
		if(clock_time_exceed(DATA2_1STimer, 1000)){			//�������¼� T=1s
			DATA2_1STimer = Gets_Clock_value();
			CAN_TxDATA2();
		}
	}
}

void FTM1_init(void)
{
	FTM_ConfigType FTM1_Config={0};

	FTM1_Config.modulo = 750 * CANTx_INTERVAL - 1;
	FTM1_Config.clk_source=FTM_CLOCK_SYSTEMCLOCK;
	FTM1_Config.prescaler=FTM_CLOCK_PS_DIV64;
	FTM1_Config.mode=1;
	FTM1_Config.toie=1;

	FTM_SetCallback(FTM1, FTM1_Task);//50ms

	FTM_Init(FTM1,&FTM1_Config);
#ifdef DEBUG
	printf("\r\nPTM1 initialization successful.\r\n");
#endif
}

void CAN_ECU_Init(void){
	FTM1_init();
}

void Can_Init(){
#ifdef CAN_CONTROL
	CAN_ECU_Init();//��ECU���ͽ���
	CAN_PC_Init();
#endif
}

#endif
