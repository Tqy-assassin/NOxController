/*
 * YUC_Y24.c
 *
 *  Created on: 2022骞�1鏈�19鏃�
 *      Author: Administrator
 */

#include "config.h"

#if VENDOR_ID == YUC_Y24
#include "YUC_Y24.h"
#include "string.h"
#include "gpio.h"
#include "device_state_manger.h"
#include "can_manage.h"
#include "sensor_control.h"
#include "can_api.h"
#include <string.h>
#include "flash_manage.h"
#include "common.h"
#include "power_manage.h"

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

static uint32_t TransmitParameterCorrectionAddress = 0;
static Error_t errorByte = {
		.byte = 0
};
static Status_t statusByte = {
		.byte = 0x40
};

void StatusPowerHasProblem(uint8_t err)
{
	statusByte.bits.SensorSupply = err != 0 ? 1 : 0;
}


void JudgeDeviceType(void)
{
#if NOXSOURCEADDR != NOXADDR_AUTO
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
        TransmitParameterCorrectionAddress = IntakeTransmitParameterCorrection;
	}else if(SourceAddr == DeviceType_Outlet){
		TransmitData = OutletTransmitData;
        TransmitParameterCorrectionAddress = OutletTransmitParameterCorrection;
	}else{

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
        if(RxFrame->Frame.code.Request1){
        	//TODO: 后NOx传感器ID反馈1
        	//需求待输入
        }
        if(RxFrame->Frame.code.Request2){
        	//TODO: 后NOx传感器ID反馈2
        	//需求待输入
        }
    }
}

#define RangeAssignment(x, a, b)	((x) >= (a) ? ((x) <= (b) ? (x) : (b)) : (a))
void CAN_TxTask()
{

    static int count = 0;
    static int TxMsgCount = 0;
    ItemInfoType sTxFrame;
	CANTxDateFrame TxFrame;

    if(CAN_Start){
        ADtfValue * ADtf_Value;
        ADtf_Value = get_ADtf_Value();
//          TxFrame.Frame.NOx = get_NOxC() * 8.9296875;
//          TxFrame.Frame.O2 = 21-get_O2C() / 0.021;
        TxFrame.Frame.NOx = RangeAssignment(get_NOxC() * 0.1, -100, 1650);
        TxFrame.Frame.O2 = RangeAssignment(get_O2C(), -200, 1540);
        TxFrame.Frame.Vp0 = ADtf_Value->Vref0Value * 100000;
        TxFrame.Frame.Status = statusByte;
        TxFrame.Frame.Error = errorByte;

        sTxFrame.ID_Type.ID = TransmitData;
        sTxFrame.bIsExtOrStand = 0;
        sTxFrame.bIsRemoteFrame = 0;
        sTxFrame.u32DataLength = 8;
        sTxFrame.u8BPR = 0x10;
        memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
        CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
    }else{
        static uint8_t DataModeSwitch = 1;
        //XXX:data2 data3 所有数据具体待确认，暂时使用GWM的数据
        switch(DataModeSwitch){ 
            case 1:
            {
                TxFrame.Frame.NOx = 0xFFFF;
                TxFrame.Frame.O2 = 0xFFFF;
                TxFrame.Frame.Vp0 = 0xFFFF;
                TxFrame.Frame.Status.byte = 0x40;
                TxFrame.Frame.Error = errorByte;
                DataModeSwitch = 2;
                break;
            }
            case 2:
            {
                const uint8_t TxData[] = {0x03, 0xDC, 0x0A, 0x00, 0x02, 0x0E, 0x0A, 0x1B};
                static uint8_t unknowncount = 0xFF;
                memcpy(TxFrame.TxData, TxData, 8);
                TxFrame.TxData[3] = unknowncount;
                unknowncount--;
                DataModeSwitch = 3;
                break;
            }
            case 3:
            {
                const uint8_t TxData[] = {0x00, 0x00, 0x00, 0x00, 0xA5, 0xD2, 0x0C, 0x00};
                memcpy(TxFrame.TxData, TxData, 8);
                DataModeSwitch = 1;
                break;
            }
        }

        sTxFrame.ID_Type.ID = TransmitData;
        sTxFrame.bIsExtOrStand = 0;
        sTxFrame.bIsRemoteFrame = 0;
        sTxFrame.u32DataLength = 8;
        sTxFrame.u8BPR = 0x10;
        memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
        CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
    }

    //TODO mode3 mode4 内容待确定


    count++;
    if(count >= 10){
        const uint8_t CAL_ID[16] = {"4A800_345_290009"};
        uint8_t CVN[4] = {0, 0, 0, 0};
        count = 0;
        memset(sTxFrame.u8DataBuff,0,8);
        sTxFrame.u8DataBuff[0] = TxMsgCount;
        if(TxMsgCount < 16){
            sTxFrame.u8DataBuff[1] = CAL_ID[TxMsgCount];
        }else{
            sTxFrame.u8DataBuff[1] = CVN[TxMsgCount-16];
        }
        TxMsgCount = (TxMsgCount + 1) % 20;

        sTxFrame.ID_Type.ID = TransmitParameterCorrectionAddress;
        sTxFrame.bIsExtOrStand = 0;
        sTxFrame.bIsRemoteFrame = 0;
        sTxFrame.u32DataLength = 8;
        sTxFrame.u8BPR = 0x10;
        CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
    }
}

void Status_HeaterOff(void){

}
void Status_PreHeater(void){

}
void Status_Heating(void){

}
void Status_HeatCriticalPoint(void){

}
void Status_AutomaticHeat(void){

}
void Status_AtTemperature(void){
	statusByte.bits.HeaterTemperature = 1;
}
void Status_nAtTemperature(void){
	statusByte.bits.HeaterTemperature = 0;
}
void Status_NOxValid(void){
    statusByte.bits.NOxSignal = 1;
}
void Status_NOxnValid(void){
    statusByte.bits.NOxSignal = 0;
}
void Status_O2Valid(void){
	statusByte.bits.LamdaLinearSignal = 1;
	statusByte.bits.LamdaBinarySignal = 1;
}
void Status_O2nValid(void){
	statusByte.bits.LamdaLinearSignal = 0;
	statusByte.bits.LamdaBinarySignal = 0;
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
	if(Rx_Buf[1] == 0x00){
		errorByte.bits.HeaterOC = 0;
		errorByte.bits.HeaterSC = 0;
	}else{
		if(Rx_Buf[3] | (1 << 5)){
			errorByte.bits.HeaterSC = 1;
		}
		if(Rx_Buf[4] | (1 << 5)){
			errorByte.bits.HeaterOC = 0;
		}
	}

	if(Rx_Buf[2] & ~(1 << 6)){
		errorByte.bits.O2OC = 0;
		errorByte.bits.O2SC = 0;
		errorByte.bits.Vp0OC = 0;
		errorByte.bits.Vp0SC = 0;
	}else{
		if(Rx_Buf[3] | (1 << 4)){
			errorByte.bits.O2SC = 1;
			errorByte.bits.Vp0SC = 1;
		}
		if(Rx_Buf[4] | (1 << 4)){
			errorByte.bits.O2OC = 0;
			errorByte.bits.Vp0OC = 0;
		}
	}

	if(Rx_Buf[2] & ~(1 << 5)){
		errorByte.bits.NOxOC = 0;
		errorByte.bits.NOxSC = 0;
	}else{
		if(Rx_Buf[3] | (1 << 4)){
			errorByte.bits.NOxSC = 1;
		}
		if(Rx_Buf[4] | (1 << 4)){
			errorByte.bits.NOxOC = 0;
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
	return errorByte.byte;
}
#endif

