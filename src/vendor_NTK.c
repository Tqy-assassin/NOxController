/*
 * vendor_NTK.c
 *
 *  Created on: 2022年1月29日
 *      Author: LiquidSource
 */

#include "config.h"
#if VENDOR_ID == NTK_KIND
#include "vendor_NTK.h"
#include "vendor.h"
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
extern uint32_t CAN_StopTimemr;	//露锟斤拷停止时锟斤拷
extern uint8_t CAN_Stop;			//露锟斤拷停止锟斤拷识

extern float Torque;
extern float Speed;
extern float SpeedAdjust;
extern float TorqueLosses;
extern float GasMassFlow;
extern uint8_t NoPIDflag;

extern uint32_t Start_Timer;

int8_t RxPackflag = 0;

J1939_SensorStatus	SensorStatus = {0};	//锟斤拷锟斤拷锟斤拷状态
void JudgeDeviceType(void)
{
//#if defined(Default_SourceAddr)
#if NOXSOURCEADDR == NOXADDR_AUTO
	CONFIG_PIN_AS_GPIO(PTA,PTA1,INPUT);
	ENABLE_INPUT(PTA, PTA1);
	if(READ_INPUT(PTA, PTA1) == 0){	//Default_SourceAddr
		SourceAddr = SourceAddrATI1;//前锟斤拷锟斤拷1锟斤拷PTA1锟接碉拷
	}else{
		SourceAddr = SourceAddrATO1;//锟斤拷锟斤拷1锟斤拷PTA1锟竭碉拷平
	}
#else
	SourceAddr = DefaultSourceAddr;
#endif

	if(SourceAddr == SourceAddrATI1){//前锟斤拷锟斤拷1
		TransmitData = TransmitDataATI1;
	}else if(SourceAddr == SourceAddrATO1){//锟斤拷锟斤拷1
		TransmitData = TransmitDataATO1;
	}else if(SourceAddr == SourceAddrATI2){//前锟斤拷锟斤拷2
		TransmitData = TransmitDataATI2;
	}else if(SourceAddr == SourceAddrATO2){//锟斤拷锟斤拷2
		TransmitData = TransmitDataATO2;
	}
}
uint8_t JudgeDewPointStart(CANRxFrameDataType* RxFrame);


void CAN_Heaterratio_deviation_Transmit(uint16_t cmdid)//data2
{
	uint16_t cmd = 0;
	uint16_t HeaterRatio,NOx_corr_gain,OperationHoursCounter;
	uint8_t NOx_corr_off;
	if(cmdid == CorrectO2NOxATI1R) cmd = HeaterRatioATI1T;
	if(cmdid == CorrectO2NOxATO1R) cmd = HeaterRatioATO1T;
	if(cmdid == CorrectO2NOxATI2R) cmd = HeaterRatioATI2T;
	if(cmdid == CorrectO2NOxATO2R) cmd = HeaterRatioATO2T;
	if(cmdid == DataAllAddr){
		switch(SourceAddr){
			case SourceAddrATI1: cmd = HeaterRatioATI1T;	break;
			case SourceAddrATO1: cmd = HeaterRatioATO1T;   break;
			case SourceAddrATI2: cmd = HeaterRatioATI2T;	break;
			case SourceAddrATO2: cmd = HeaterRatioATO2T;	break;
			default:  break;
		}
	}
	ItemInfoType sTxFrameInfo;
	HeaterRatioTFrame_TypeDef CTxFrame;
	uint32_t timer;
	uint8_t Status;
	memset(&CTxFrame,0,8);
	sTxFrameInfo.ID_Type.ID = PDU_P(3) | PDU_R(0) | PDU_DP(0) | PDU_PGN(cmd) | PDU_SA(SourceAddr);//CFD 1051  //锟斤拷锟饺硷拷P3位锟斤拷锟斤拷锟斤拷位R1位锟斤拷锟斤拷锟斤拷页DP1位锟斤拷PGN锟斤拷PF+PS锟斤拷16位锟斤拷源锟斤拷址SA8位
	sTxFrameInfo.bIsExtOrStand = 1;
	sTxFrameInfo.bIsRemoteFrame = 0;
	sTxFrameInfo.u32DataLength = 8;
	sTxFrameInfo.u8BPR = 0x10;

	HeaterRatio = 1/0.001;
	NOx_corr_gain = (0+100)/0.1;
	NOx_corr_off = (0+125)/1;
	OperationHoursCounter = get_new_run_time()/3600/1;//runtime (h)
	//锟斤拷锟斤拷
	CTxFrame.TxFrame.HeaterRatio_L = HeaterRatio & 0xFF;
	CTxFrame.TxFrame.HeaterRatio_H = HeaterRatio>>8 & 0xFF;
	CTxFrame.TxFrame.NOx_corr_gain_L = NOx_corr_gain & 0xFF;
	CTxFrame.TxFrame.NOx_corr_gain_H = NOx_corr_gain>>8 & 0xFF;
	CTxFrame.TxFrame.NOx_corr_off = NOx_corr_off;
	CTxFrame.TxFrame.OperationHoursCounter_L = OperationHoursCounter & 0xFF;
	CTxFrame.TxFrame.OperationHoursCounter_H = OperationHoursCounter>>8 & 0xFF;
	//CTxFrame.TxFrame.Noused = 0xFF;

	memcpy(sTxFrameInfo.u8DataBuff, CTxFrame.TxData, 8);
	timer = Gets_Clock_value();
	do{
		Status = CAN_TransmitItemByInt(MSCAN,&sTxFrameInfo,&sCAN_TxBuff);
		if(clock_time_exceed(timer,500)){
			break;
		}
	}while(!Status);
}



void CAN_DeclareAddress()
{
	DelayUS(10000);
	//uint32_t timer;
	//uint8_t Status;
	uint16_t cmd = 0xEEFF;

	ItemInfoType sTxFrameInfo;
	GeneralTFrame_TypeDef CTxFrame;
	memset(&CTxFrame,0,8);
	//3+1+1+16+8 = 29
	sTxFrameInfo.ID_Type.ID = PDU_P(6) | PDU_R(0) | PDU_DP(0) | PDU_PGN(cmd) | PDU_SA(SourceAddr);//CFD 1051  //锟斤拷锟饺硷拷P3位锟斤拷锟斤拷锟斤拷位R1位锟斤拷锟斤拷锟斤拷页DP1位锟斤拷PGN锟斤拷PF+PS锟斤拷16位锟斤拷源锟斤拷址SA8位
	sTxFrameInfo.bIsExtOrStand = 1;
	sTxFrameInfo.bIsRemoteFrame = 0;
	sTxFrameInfo.u32DataLength = 8;
	sTxFrameInfo.u8BPR = 0x10;

	//锟斤拷锟斤拷  data
	CTxFrame.TxFrame.byte0Data = 0x80;
	if(SourceAddr == SourceAddrATI1){
		CTxFrame.TxFrame.byte1Data = 0x11;
	}
	if(SourceAddr == SourceAddrATO1){
		CTxFrame.TxFrame.byte1Data = 0x21;
	}
	CTxFrame.TxFrame.byte2Data = 0x52;
	CTxFrame.TxFrame.byte3Data = 0x16;
	CTxFrame.TxFrame.byte4Data = 0x00;
	CTxFrame.TxFrame.byte5Data = 0x44;
	CTxFrame.TxFrame.byte6Data = 0x00;
	CTxFrame.TxFrame.byte7Data = 0x01;

	memcpy(sTxFrameInfo.u8DataBuff, CTxFrame.TxData, 8);
	CAN_TransmitItemByInt(MSCAN,&sTxFrameInfo,&sCAN_TxBuff);
	/*
	timer = Gets_Clock_value();
	do{
		Status = CAN_TransmitItemByInt(MSCAN,&sTxFrameInfo,&sCAN_TxBuff);
		if(clock_time_exceed(timer,500)){
			break;
		}
	}while(!Status);
	 */
	DelayUS(10000);
}


void CAN_CorrectO2NOx_Transmit(uint16_t cmdid)//data3
{
	uint16_t cmd = 0;
	CalculateCoeffi_Typedef* PCoe = Coeffi_pointer();
	if(cmdid == CorrectO2NOxATI1R) cmd = CorrectO2NOxATI1T;
	if(cmdid == CorrectO2NOxATO1R) cmd = CorrectO2NOxATO1T;
	if(cmdid == CorrectO2NOxATI2R) cmd = CorrectO2NOxATI2T;
	if(cmdid == CorrectO2NOxATO2R) cmd = CorrectO2NOxATO2T;
	if(cmdid == DataAllAddr){
		switch(SourceAddr){
			case SourceAddrATI1: cmd = CorrectO2NOxATI1T;	break;
			case SourceAddrATO1: cmd = CorrectO2NOxATO1T;   break;
			case SourceAddrATI2: cmd = CorrectO2NOxATI2T;	break;
			case SourceAddrATO2: cmd = CorrectO2NOxATO2T;	break;
			default:  break;
		}
	}

	ItemInfoType sTxFrameInfo;
	CorrectO2NOxTFrame_TypeDef CTxFrame;
	uint32_t timer;
	uint8_t Status;
	memset(&CTxFrame,0,8);
	//3+1+1+16+8 = 29
	sTxFrameInfo.ID_Type.ID = PDU_P(3) | PDU_R(0) | PDU_DP(0) | PDU_PGN(cmd) | PDU_SA(SourceAddr);//CFD 1051  //锟斤拷锟饺硷拷P3位锟斤拷锟斤拷锟斤拷位R1位锟斤拷锟斤拷锟斤拷页DP1位锟斤拷PGN锟斤拷PF+PS锟斤拷16位锟斤拷源锟斤拷址SA8位
	sTxFrameInfo.bIsExtOrStand = 1;
	sTxFrameInfo.bIsRemoteFrame = 0;
	sTxFrameInfo.u32DataLength = 8;
	sTxFrameInfo.u8BPR = 0x10;

	//锟斤拷锟斤拷  data
	CTxFrame.TxFrame.SelfDiagnosisResultValue = 100/1;
	CTxFrame.TxFrame.NH3Correction = 0/0.005;
	CTxFrame.TxFrame.NO2Correction = 0.85/0.005;
	CTxFrame.TxFrame.CorrectPressureLambda = (PCoe->O2_Pressure)/0.005;		//锟斤拷压锟斤拷锟斤拷锟斤拷系锟斤拷0.36
	CTxFrame.TxFrame.CorrectPressureNOx = (PCoe->NOx_Pressure)/0.005;		//锟斤拷锟斤拷压锟斤拷锟斤拷锟斤拷系锟斤拷0.11

	memcpy(sTxFrameInfo.u8DataBuff, CTxFrame.TxData, 8);

	timer = Gets_Clock_value();
	do{
		Status = CAN_TransmitItemByInt(MSCAN,&sTxFrameInfo,&sCAN_TxBuff);
		if(clock_time_exceed(timer,500)){
			break;
		}
	}while(!Status);
}

void CAN_TriggerSelfDiagnosis(uint16_t cmdid,CANRxFrameDataType* RxFrame){

	if(CAN_Start){
		if(!clock_time_exceed(Start_Timer,120 * 1000)){			//锟斤拷锟饺猴拷120s锟斤拷
			if( (cmdid == TransmitDataATI1)||\
				(cmdid == TransmitDataATO1)||\
				(cmdid == TransmitDataATI2)||\
				(cmdid == TransmitDataATO2)   ){
				uint8_t temp = 0;
				switch(cmdid){
					case TransmitDataATI1: temp = RxFrame->RxData[0] & 0x0F;	break;
					case TransmitDataATO1: temp = RxFrame->RxData[0]>>4 & 0x0F;break;
					case TransmitDataATI2: temp = RxFrame->RxData[1] & 0x0F;	break;
					case TransmitDataATO2: temp = RxFrame->RxData[1]>>4 & 0x0F;break;
					default:  break;
				}
				if((temp <= 4) && (temp >= 1))
					TriggerSelfDiagnosis = 1;//锟斤拷锟斤拷锟侥Ｊ斤拷锟斤拷锟�
				else
					TriggerSelfDiagnosis = 0;//锟斤拷锟斤拷锟侥Ｊ斤拷乇锟�
			}
		}else{
			TriggerSelfDiagnosis = 0;//锟斤拷锟斤拷锟侥Ｊ斤拷乇锟�
		}
	}
}


void CAN_RxManyPostback(){
	uint32_t PGN;
	if(RxPackflag > 0){
		RxPackflag--;
		switch(SourceAddr){
		case SourceAddrATI1:
			PGN = 0xEA51;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
			break;
		case SourceAddrATO1:
			PGN = 0xEA52;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
			break;
		case CorrectO2NOxATI2R:
			PGN = 0xEA56;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
			break;
		case CorrectO2NOxATO2R:
			PGN = 0xEA57;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
			break;
		case DataAllAddr:
			PGN = 0xEAFF;
			CAN_CorrectO2NOx_Transmit(PGN);
			CAN_Heaterratio_deviation_Transmit(PGN);
			break;
		default:
			break;
		}
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
		Torque = getTorque(RxFrame);
		Speed = getSpeed(RxFrame);
		SpeedAdjust = getSpeedAdjust(RxFrame);
		TorqueLosses = getTorqueLosses(RxFrame);
		GasMassFlow = getGasMassFlow(RxFrame);
		break;
	case CorrectO2NOxATI1R:
		if(SourceAddr == SourceAddrATI1){
			RxPackflag = 4;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
		}
		break;
		/*
		if(SourceAddr == SourceAddrATI1){
			for(int i = 0; i < 5; i++){
				CAN_CorrectO2NOx_Transmit(PGN);//data3
				CAN_Heaterratio_deviation_Transmit(PGN);//data2
			}
		}
		break;*/
	case CorrectO2NOxATO1R:
		if(SourceAddr == SourceAddrATO1){
			RxPackflag = 4;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
		}
		break;
	case CorrectO2NOxATI2R:
		if(SourceAddr == SourceAddrATI2){
			RxPackflag = 4;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
		}
		break;
	case CorrectO2NOxATO2R:
		if(SourceAddr == SourceAddrATO2){
			RxPackflag = 4;
			CAN_CorrectO2NOx_Transmit(PGN);//data3
			CAN_Heaterratio_deviation_Transmit(PGN);//data2
		}
		break;
	case TriggerforSelfDiagnosis:
		CAN_TriggerSelfDiagnosis(TransmitData,RxFrame);
		break;
	case DataAllAddr:
		CAN_CorrectO2NOx_Transmit(PGN);
		CAN_Heaterratio_deviation_Transmit(PGN);
		break;
	default:
		break;
	}
}

uint8_t JudgeDewPointStart(CANRxFrameDataType* RxFrame)
{
	if((RxFrame->RxFrame.startCode.AfterTreatment1IntakeGas == DewPointReached && SourceAddr == SourceAddrATI1)
				|| (RxFrame->RxFrame.startCode.AfterTreatment1OutletGas == DewPointReached && SourceAddr == SourceAddrATO1)
				|| (RxFrame->RxFrame.startCode.AfterTreatment2IntakeGas == DewPointReached && SourceAddr == SourceAddrATI2)
				|| (RxFrame->RxFrame.startCode.AfterTreatment2OutletGas == DewPointReached && SourceAddr == SourceAddrATO2)){
		return DewPoint_Start;
	}else if((RxFrame->RxFrame.startCode.AfterTreatment1IntakeGas == DewPointNoReached && SourceAddr == SourceAddrATI1)
			|| (RxFrame->RxFrame.startCode.AfterTreatment1OutletGas == DewPointNoReached && SourceAddr == SourceAddrATO1)
			|| (RxFrame->RxFrame.startCode.AfterTreatment2IntakeGas == DewPointNoReached && SourceAddr == SourceAddrATI2)
			|| (RxFrame->RxFrame.startCode.AfterTreatment2OutletGas == DewPointNoReached && SourceAddr == SourceAddrATO2)){
		return DewPoint_Stop;
	}else{
		return FALSE;
	}
}
float getTorque(CANRxFrameDataType* RxFrame){
	return RxFrame->RxFrame.Torque - 125;
}
float getSpeed(CANRxFrameDataType* RxFrame){
	return ((uint16_t)(RxFrame->RxFrame.Speed_L) | (RxFrame->RxFrame.Speed_H << 8)) * 0.125;
}
float getSpeedAdjust(CANRxFrameDataType* RxFrame){
	return RxFrame->RxFrame.SpeedAdjusting;
}
float getTorqueLosses(CANRxFrameDataType* RxFrame){
	return RxFrame->RxFrame.TorqueLosses - 125;
}
float getGasMassFlow(CANRxFrameDataType* RxFrame){
	return ((uint16_t)(RxFrame->RxFrame.AfterTreatment1ExhaustGasMassFlow_L) | (RxFrame->RxFrame.AfterTreatment1ExhaustGasMassFlow_H << 8)) * 0.2;
}

void CAN_TxDATA2(void)
{
	uint32_t PGN;
	if(!CAN_Start){
		switch(SourceAddr){
			case SourceAddrATI1: PGN = CorrectO2NOxATI1R;break;
			case SourceAddrATO1: PGN = CorrectO2NOxATO1R;break;
			case SourceAddrATI2: PGN = CorrectO2NOxATI2R;break;
			case SourceAddrATO2: PGN = CorrectO2NOxATO2R;break;
			default:  PGN = CorrectO2NOxATI1R;break;
		}
		CAN_Heaterratio_deviation_Transmit(PGN);
	}
}



void CAN_TxTask(void)
{
	ItemInfoType sTxFrame;
	TramsmitFrame_TypeDef TxFrame;
	stage workingstage = get_working_stage();
	if(CAN_Start){
		TxFrame.TxFrame.O2 = (uint16_t)(((get_O2C()) + 12)/0.0005135);
		TxFrame.TxFrame.O2 = TxFrame.TxFrame.O2 > 0xFAFF ? 0xFAFF : TxFrame.TxFrame.O2;
		TxFrame.TxFrame.NOx = (uint16_t)(((get_NOxC()) + 200)/0.05);
		TxFrame.TxFrame.NOx = TxFrame.TxFrame.NOx > 0xFAFF ? 0xFAFF : TxFrame.TxFrame.NOx;
		TxFrame.TxFrame.StatusByte.NOxSignal = (SensorStatus.IfNOxValid) ? NOxsignalValid : NOxsignalnValid;
		TxFrame.TxFrame.StatusByte.O2Signal = (SensorStatus.IfO2Valid) ? O2signalValid : O2signalnValid;
		TxFrame.TxFrame.StatusByte.TempHeaterElement = SensorStatus.IfAtTemperature ? SensorAtTemper : SensornAtTemper;
		TxFrame.TxFrame.StatusByte.SupplyinRange = SupplyInRange;

		TxFrame.TxFrame.StatusHeaterMode = SensorStatus.HeaterMode;
		TxFrame.TxFrame.ErrorHeater =  (SensorStatus.HeaterError) ? (SensorStatus.HeaterError > 1 ? OpenWire : ShortCircuit) : NoError;
		TxFrame.TxFrame.ErrorNOx = (SensorStatus.NOxError) ? (SensorStatus.NOxError > 1 ? OpenWire : ShortCircuit) : NoError;
		TxFrame.TxFrame.ErrorO2 = (SensorStatus.O2Error) ? (SensorStatus.O2Error > 1 ? OpenWire : ShortCircuit) : NoError;


		if(TriggerSelfDiagnosis){//锟斤拷锟斤拷锟�
			if(workingstage == STAGE_ENVIRONMENT){
				TxFrame.TxFrame.Diagnosisfeedback = 2;
			}
			else{
				TxFrame.TxFrame.Diagnosisfeedback = 1;
			}
		}
		else {
			TxFrame.TxFrame.Diagnosisfeedback = 0;
		}

	}else{
		memset(TxFrame.TxData,0xFF,8);
		/*
		TxFrame.TxFrame.O2 = 0xFFFF;
		TxFrame.TxFrame.NOx = 0xFFFF;
		TxFrame.TxFrame.StatusByte.NOxSignal = 0x3;
		TxFrame.TxFrame.StatusByte.O2Signal = 0x3;
		TxFrame.TxFrame.StatusByte.TempHeaterElement = 0x3;
		TxFrame.TxFrame.StatusByte.SupplyinRange = 0x3;
		TxFrame.TxFrame.StatusHeaterMode = 0x3;
		TxFrame.TxFrame.ErrorHeater = 0x1F;
		TxFrame.TxFrame.ErrorNOx = 0x1F;
		TxFrame.TxFrame.ErrorO2 = 0x1F;
		TxFrame.TxFrame.Diagnosisfeedback = 1;
		*/
	}

	sTxFrame.ID_Type.ID = PDU_P(6) | PDU_R(0) | PDU_DP(0) | PDU_PGN(TransmitData) | PDU_SA(SourceAddr);//锟斤拷锟饺硷拷P3位锟斤拷锟斤拷锟斤拷位R1位锟斤拷锟斤拷锟斤拷页DP1位锟斤拷PGN锟斤拷PF+PS锟斤拷16位锟斤拷源锟斤拷址SA8位
	sTxFrame.bIsExtOrStand = 1;//锟斤拷展帧
	sTxFrame.bIsRemoteFrame = 0;//锟斤拷远锟斤拷帧锟斤拷锟斤拷锟斤拷锟斤拷帧
	sTxFrame.u32DataLength = 8;//锟斤拷锟捷筹拷锟斤拷8锟街斤拷
	sTxFrame.u8BPR = 0x10;
	memcpy(sTxFrame.u8DataBuff,TxFrame.TxData,8);
	CAN_TransmitItemByInt(MSCAN,&sTxFrame,&sCAN_TxBuff);
//	J1939_Transmit(MSCAN,&TxFrame);
}

void Status_HeaterOff(void){
	SensorStatus.HeaterMode = HeaterOfforPre;
}
void Status_PreHeater(void){
	SensorStatus.HeaterMode = HeaterOfforPre;
}
void Status_Heating(void){
	SensorStatus.HeaterMode = HeatupSlope1or2;
}
void Status_HeatCriticalPoint(void){
	SensorStatus.HeaterMode = HeatupSlope3or4;
}
void Status_AutomaticHeat(void){
	SensorStatus.HeaterMode = AutomaticMode;
}
void Status_AtTemperature(void){
	SensorStatus.IfAtTemperature = 1;
}
void Status_nAtTemperature(void){
	SensorStatus.IfAtTemperature = 0;
}
void Status_NOxValid(void){
	SensorStatus.IfNOxValid = NOxsignalValid;
}
void Status_NOxnValid(void){
	SensorStatus.IfNOxValid = NOxsignalnValid;
}
void Status_O2Valid(void){
	SensorStatus.IfO2Valid = O2signalValid;
}
void Status_O2nValid(void){
	SensorStatus.IfO2Valid = O2signalnValid;
}
uint8_t Status_Get(void){
	StatusByte_t Status;
	Status.status.NOxSignal = (SensorStatus.IfNOxValid) ? NOxsignalValid : NOxsignalnValid;
	Status.status.O2Signal = (SensorStatus.IfO2Valid) ? O2signalValid : O2signalnValid;
	Status.status.TempHeaterElement = SensorStatus.IfAtTemperature ? SensorAtTemper : SensornAtTemper;
	Status.status.SupplyinRange = SupplyInRange;
	return Status.byte;
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
			Status &= ~(1<<i);//锟斤拷锟斤拷锟斤拷锟斤拷一位为0
		}else{
			Status |= 1<<i;//锟斤拷锟斤拷锟斤拷锟斤拷一位为1
		}
	}

	if(Status > 1){
		if((get_working_stage() != STAGE_IDLE) && (get_working_stage() != STAGE_InspectError)){
			if(count > 200){
#ifdef UART_CONTROL
				SAVEPOS();
				MOVETO(16,0);
				printf("Error: 0x%02X 0x%02X 0x%02X 0x%02X",Rx_Buf[1],Rx_Buf[2],Rx_Buf[3],Rx_Buf[4]);
				LODEPOS();
#endif
				//set_pre_working_stage(get_working_stage());
				set_working_stage(STAGE_InspectError);
				buf[0] = Ip->Ip0_Value;
				buf[1] = Ip->Ip1_Value;
				buf[2] = Ip->Ip2_Value;
				init_Ip_Value();
			}else{
				count++;
			}
		}
		if(Status & InspectBit1){
			if(Rx_Buf[1] & InspectBit7){
				SensorStatus.HeaterError |= 3;
			}
			if(Rx_Buf[1] & InspectBit6){
				SensorStatus.HeaterError |= 3;
			}
			if(Rx_Buf[1] & InspectBit5){
				SensorStatus.HeaterError |= 3;
			}
		}
		if(Status && InspectBit2){
			if(Rx_Buf[2] & InspectBit7){
				SensorStatus.NOxError |= 3;
			}
			if(Rx_Buf[1] & InspectBit6){
				SensorStatus.O2Error |= 3;
			}
		}
		if(Status & InspectBit3){
			if(Rx_Buf[3] & InspectBit5){
				SensorStatus.HeaterError |= 1;
			}else if(Rx_Buf[3] & InspectBit4){
				SensorStatus.O2Error |= 1;
			}else if(Rx_Buf[3] & InspectBit3){
			}else if(Rx_Buf[3] & InspectBit2){
				SensorStatus.NOxError |= 1;
			}else if(Rx_Buf[3] & InspectBit1){
			}else if(Rx_Buf[3] & InspectBit0){
			}
		}
		if(Status & InspectBit4){
			if(Rx_Buf[3] & InspectBit5){
				SensorStatus.HeaterError |= 2;
			}else if(Rx_Buf[3] & InspectBit4){
				SensorStatus.O2Error |= 2;
			}else if(Rx_Buf[3] & InspectBit3){
			}else if(Rx_Buf[3] & InspectBit2){
				SensorStatus.NOxError |= 2;
			}else if(Rx_Buf[3] & InspectBit1){
			}else if(Rx_Buf[3] & InspectBit0){
			}
		}
	}else{
		if(count == 0){
			SensorStatus.NOxError = 0;
			SensorStatus.O2Error = 0;
			SensorStatus.HeaterError = 0;
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
#endif /* VENDOR_ID == NTK_KIND */
