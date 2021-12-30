/*
 * gwm_vendor.c
 *
 *  Created on: 2021年12月15日
 *      Author: Administrator
 */

#include "gwm_vendor.h"
#include "string.h"
#include "gpio.h"
#include "device_state_manger.h"
#include "can_manage.h"
#include "sensor_control.h"
#include "can_api.h"
#include <string.h>
#include "flash_manage.h"
#include "common.h"

#if VENDOR_ID == GWM_KIND
uint32_t SourceAddr = 0;
uint32_t TransmitData = 0;

uint8_t TriggerSelfDiagnosis = 0;

extern uint32_t CAN_StartTimemr;
extern uint8_t CAN_Start;

extern uint32_t StartTimer;
extern uint32_t CAN_StopTimemr;
extern uint8_t CAN_Stop;

extern float Torque;
extern float Speed;
extern float SpeedAdjust;
extern float TorqueLosses;
extern float GasMassFlow;
extern uint8_t NoPIDflag;

extern uint32_t Start_Timer;
int8_t RxPackflag = 0;

static uint32_t TransmitParameterCorrection1Address = 0;
static uint32_t TransmitParameterCorrection2Address = 0;
static ErrorState errorState = {
		.StateDate = 0
};


void JudgeDeviceType(void)
{
#if defined(Default_SourceAddr)
	SourceAddr = DefaultSourceAddr;	//Default_SourceAddr
#else
	CONFIG_PIN_AS_GPIO(PTA,PTA1,INPUT);
	ENABLE_INPUT(PTA, PTA1);
	if(READ_INPUT(PTA, PTA1) == 0){
		SourceAddr = DeviceType_Intake;
	}else{
		SourceAddr = DeviceType_Outlet;
	}
#endif

	if(SourceAddr == DeviceType_Intake){
		TransmitData = IntakeTransmitData;
        TransmitParameterCorrection1Address = IntakeTransmitParameterCorrection1;
        TransmitParameterCorrection2Address = IntakeTransmitParameterCorrection2;
	}else if(SourceAddr == DeviceType_Outlet){
		TransmitData = OutletTransmitData;
        TransmitParameterCorrection1Address = OutletTransmitParameterCorrection1;
        TransmitParameterCorrection2Address = OutletTransmitParameterCorrection2;
	}
}

void CAN_RxTask(uint32_t CAN_ID, CANRxFrameDataType* RxFrame)
{
    if((CAN_ID == OutletReceiveSignal && SourceAddr == DeviceType_Outlet) 
    || (CAN_ID == IntakeReceiveSignal && SourceAddr == DeviceType_Intake)){
        if(RxFrame->Frame.code.DewPoint == 1){
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
        }else{
            CAN_StopTimemr = Gets_Clock_value();
            CAN_Stop = 1;
        }
    }
}
const uint8_t TxMsg0Bit[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13};
const uint8_t TxMsg1Bit[] = {0x30,0x36,0x31,0x36,0x30,0x35,0x30,0x2D,0x46,0x44,0x32,0x39,0x30,0x30,0x34,0x33,0x3D,0x79,0x48,0xA8};
void CAN_TxTask()
{
    static int count = 0;
    static int TxMsgCount = 0;
    ItemInfoType sTxFrame;
	CANTxDateFrame TxFrame;

	if(CAN_Start){
        ADtfValue * ADtf_Value;
        ADtf_Value = get_ADtf_Value();
        TxFrame.Frame.NOx = get_NOxC() * 8.9296875;
        TxFrame.Frame.O2 = 21-get_O2C() / 0.021;
        TxFrame.Frame.Vp0 = ADtf_Value->Vref0Value * 100000;
        TxFrame.Frame.Rst = 30;
        TxFrame.Frame.Error = errorState;
	}else{
		static int CAN_StartCount = 0;
        switch(CAN_StartCount){
            case 0:
            {
                uint8_t TxData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x40, 0x00};
                memcpy(TxFrame.TxData, TxData, 8);
                CAN_StartCount = 1;
                break;
            }
            case 1:
            {
                static uint8_t unknowncount = 0xFF;
                uint8_t TxData[] = {0x03, 0xDC, 0x0A, unknowncount, 0x02, 0x0E, 0x0A, 0x1B};
                memcpy(TxFrame.TxData, TxData, 8);
                unknowncount--;
                CAN_StartCount = 2;
                break;
            }
            case 2:
            {
                uint8_t TxData[] = {0x00, 0x00, 0x00, 0x00, 0xA5, 0xD2, 0x0C, 0x00};
                memcpy(TxFrame.TxData, TxData, 8);
                CAN_StartCount = 0;
                break;
            }
        }
	}

	sTxFrame.ID_Type.ID = TransmitData;
	sTxFrame.bIsExtOrStand = 0;
	sTxFrame.bIsRemoteFrame = 0;
	sTxFrame.u32DataLength = 8;
	sTxFrame.u8BPR = 0x10;
	memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
	CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);


    count++;
    if(count >= 10){
        count = 0;
        memset(sTxFrame.u8DataBuff,0,8);
        sTxFrame.u8DataBuff[0] = TxMsg0Bit[TxMsgCount];
        sTxFrame.u8DataBuff[1] = TxMsg1Bit[TxMsgCount];
        TxMsgCount = (TxMsgCount + 1) % 20;

        sTxFrame.ID_Type.ID = TransmitParameterCorrection1Address;
        sTxFrame.bIsExtOrStand = 0;
        sTxFrame.bIsRemoteFrame = 0;
        sTxFrame.u32DataLength = 8;
        sTxFrame.u8BPR = 0x10;
        CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
    }
}

void Status_HeaterOff(void){
    errorState.State.HeaterSC = 1;
}
void Status_PreHeater(void){
	errorState.State.HeaterSC = 1;
}
void Status_Heating(void){
	errorState.State.HeaterSC = 1;
}
void Status_HeatCriticalPoint(void){
	errorState.State.HeaterSC = 1;
}
void Status_AutomaticHeat(void){
	
}
void Status_AtTemperature(void){
	errorState.State.HeaterOC = 0;
    errorState.State.HeaterSC = 0;
}
void Status_nAtTemperature(void){
	errorState.State.HeaterOC = 1;
}
void Status_NOxValid(void){
    errorState.State.NOxOC = 0;
    errorState.State.NOxSC = 0;
}
void Status_NOxnValid(void){
	errorState.State.NOxOC = 1;
}
void Status_O2Valid(void){
	errorState.State.O2OC = 0;
    errorState.State.O2SC = 0;
    errorState.State.Vp0OC = 0;
    errorState.State.Vp0SC = 0;
}
void Status_O2nValid(void){
	errorState.State.O2OC = 1;
    errorState.State.Vp0OC = 1;
}

void InspectResultAnalysis(uint8_t* InspectResult)
{
	IpValue * Ip = get_Ip_Value();
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
				buf[0] = Ip->Ip0_Value;
				buf[1] = Ip->Ip1_Value;
				buf[2] = Ip->Ip2_Value;
				init_Ip_Value();
			}else{
				count++;
			}
		}
	}else{
		if(count == 0){
			if(get_working_stage() == STAGE_InspectError){
				set_working_stage(get_pre_working_stage());
				Ip->Ip0_Value = buf[0];
				Ip->Ip1_Value = buf[1];
				Ip->Ip2_Value = buf[2];
			}
		}else if(count > 0){
			count --;
		}
	}
}
uint8_t Status_Get(void){
	return errorState.StateDate;
}
#endif
