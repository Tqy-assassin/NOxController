/*
 * vendor_BENZ.c
 *
 *  Created on: 2022年1月29日
 *      Author: LiquidSource
 */


#include "config.h"
#if VENDOR_ID == BENZ_KIND

#include "vendor_BENZ.h"
#include "string.h"
#include "gpio.h"
#include "device_state_manger.h"
#include "can_manage.h"
#include "sensor_control.h"
#include "can_api.h"
#include <string.h>
#include "flash_manage.h"
#include "common.h"

uint32_t SourceAddr = 0;
uint32_t TransmitData = 0;

uint8_t TriggerSelfDiagnosis = 0;

extern uint32_t CAN_StartTimemr;
extern uint8_t CAN_Start;

extern uint32_t StartTimer;
extern uint32_t CAN_StopTimemr;	//¶��ֹͣʱ��
extern uint8_t CAN_Stop;			//¶��ֹͣ��ʶ

extern float Torque;
extern float Speed;
extern float SpeedAdjust;
extern float TorqueLosses;
extern float GasMassFlow;
extern uint8_t NoPIDflag;

extern uint32_t Start_Timer;

int8_t RxPackflag = 0;

void JudgeDeviceType(void)
{
#if defined(Default_SourceAddr)
	SourceAddr = DefaultSourceAddr;
#else
	CONFIG_PIN_AS_GPIO(PTA,PTA1,INPUT);
	ENABLE_INPUT(PTA, PTA1);
	if(READ_INPUT(PTA, PTA1) == 0){	//Default_SourceAddr
		SourceAddr = DefaultSourceAddr;
	}else{
		SourceAddr = DefaultSourceAddr;
	}
#endif
	TransmitData = TransmitDataATI1;
}

uint8_t JudgeDewPointStart(CANRxFrameDataType* RxFrame)
{
	if((RxFrame->RxFrame.startCode.AfterTreatment1IntakeGas == DewPointReached)){
		return DewPoint_Start;
	}else{
		return DewPoint_Stop;
	}
}

void CAN_RxTask(uint32_t PGN, CANRxFrameDataType* RxFrame)
{

	switch(PGN){
	case ReceiveSignal:
		switch(JudgeDewPointStart(RxFrame)){
			case DewPoint_Start:
#if	NORAML_LAUNCH_ENABLE
				CAN_Stop = 0;
				CAN_StopTimemr = 0;
				CAN_StartTimemr = Gets_Clock_value();
				CAN_Start = 1;
				if(get_working_stage() == STAGE_IDLE){
					NoPIDflag = 1;
					set_working_stage(STAGE_PREHEATING);
					StartTimer = Gets_Clock_value();
					Start_Timer = Gets_Clock_value();
					stop_NOValueOffsetTimer();
				}
#endif
				break;
			case DewPoint_Stop:
				CAN_StopTimemr = Gets_Clock_value();
				CAN_Stop = 1;
				break;
			default:
				break;
		}
		break;
	default:
		break;
	}
}

const uint8_t TxMsg0Bit[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13};
const uint8_t TxMsg1Bit[] = {0x30,0x30,0x30,0x39,0x30,0x35,0x33,0x35,0x30,0x33,0x32,0x39,0x30,0x30,0x31,0x31,0xA6,0x88,0x38,0x1D};
void CAN_TxTask(void){
	TramsmitFrame_TypeDef TxFrame;
	ItemInfoType sTxFrame;
	uint16_t O2, NOx;
	static uint8_t Count = 0;
	static uint8_t MsgNum = 0;

	O2 = (int)((21 - get_O2C()) / 0.021);
	NOx = (int)(get_NOxC());
	memset(TxFrame.TxData, 0, 8);
	TxFrame.TxFrame.NOx[0] = (NOx >> 8) & 0xFF;
	TxFrame.TxFrame.NOx[1] = (NOx >> 0) & 0xFF;
	TxFrame.TxFrame.O2[0] = (O2 >> 8) & 0xFF;
	TxFrame.TxFrame.O2[1] = (O2 >> 0) & 0xFF;
	TxFrame.TxFrame.UNKnow[0] = 0;
	TxFrame.TxFrame.UNKnow[1] = 0;
	if(get_working_stage() == STAGE_ENVIRONMENT){
		TxFrame.TxFrame.Status = 0x1E;
	}else{
		TxFrame.TxFrame.Status = 0x00;
	}
	sTxFrame.ID_Type.ID = TransmitData;
	sTxFrame.bIsExtOrStand = 0;
	sTxFrame.bIsRemoteFrame = 0;
	sTxFrame.u32DataLength = 8;
	sTxFrame.u8BPR = 0x10;
	memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
	CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
	Count++;
	if(Count == 10){
		Count = 0;

		TxFrame.TxData[0] = TxMsg0Bit[MsgNum];
		TxFrame.TxData[1] = TxMsg1Bit[MsgNum];
		TxFrame.TxData[2] = 0;
		TxFrame.TxData[3] = 0;
		TxFrame.TxData[4] = 0;
		TxFrame.TxData[5] = 0;
		TxFrame.TxData[6] = 0;
		TxFrame.TxData[7] = 0;

		sTxFrame.ID_Type.ID = TransmitMessage;
		sTxFrame.bIsExtOrStand = 0;
		sTxFrame.bIsRemoteFrame = 0;
		sTxFrame.u32DataLength = 8;
		sTxFrame.u8BPR = 0x10;
		memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
		CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
		MsgNum++;
		MsgNum %= sizeof(TxMsg0Bit);
	}
}


void InspectResultAnalysis(uint8_t* InspectResult)
{
	IpValue *ip_Value = get_Ip_Value();
	static float buf[3];
	static int count = 0;
	uint32_t Status = 0;
	const uint8_t NormalValue[] = {0x08,0x00,0x00,0x00,0x80};
	int i;
	uint8_t Rx_Buf[5];
	memcpy(Rx_Buf,InspectResult,5);

	for(i = 0;i<5;i++){
		if(Rx_Buf[i] == NormalValue[i]){
			Status &= ~(1<<i);
		}else{
			Status |= 1<<i;
		}
	}

	if(Status > 1){
		if(get_working_stage() != STAGE_IDLE && get_working_stage() != STAGE_InspectError){
			if(count > 200){
#ifdef UART_CONTROL
				SAVEPOS();
				MOVETO(16,0);
				printf("Error: 0x%02X 0x%02X 0x%02X 0x%02X",Rx_Buf[1],Rx_Buf[2],Rx_Buf[3],Rx_Buf[4]);
				LODEPOS();
#endif
				set_working_stage(STAGE_InspectError);
				buf[0] = ip_Value->Ip0_Value;
				buf[1] = ip_Value->Ip1_Value;
				buf[2] = ip_Value->Ip2_Value;
				ip_Value->Ip0_Value = ip_Value->Ip1_Value = ip_Value->Ip2_Value = 0;
			}else{
				count++;
			}
		}
	}else{
		if(count == 0){
			if(get_working_stage() == STAGE_InspectError){
				set_working_stage(get_pre_working_stage());
				ip_Value->Ip0_Value = buf[0];
				ip_Value->Ip1_Value = buf[1];
				ip_Value->Ip2_Value = buf[2];
			}
		}else if(count > 0){
			count --;
		}
	}
}

#endif /* VENDOR_ID == BENZ_KIND */
