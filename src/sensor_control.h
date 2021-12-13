/*
 * sensor_control.h
 *
 *  Created on: 2021Äê4ÔÂ1ÈÕ
 *      Author: sunkaixiang
 */

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_

typedef struct IpValue {
	float Ip0_Value;
	float Ip1_Value;
	float Ip2_Value;
}IpValue;

typedef struct ADtfValue{
	float Vref0Value;
	float Vref1Value;
	float Vref2Value;
	float VCommValue;
	float VTempValue;
	float VTestValue;
}ADtfValue;

typedef struct KalmanADtfValue{
	float KalmanVref0Value;
	float KalmanVref1Value;
	float KalmanVref2Value;
	float KalmanVCommValue;
	float KalmanVTempValue;
	float KalmanVTestValue;
}KalmanADtfValue;

IpValue * get_Ip_Value(void);
ADtfValue * get_ADtf_Value(void);
KalmanADtfValue * get_KalmanADtf_Value(void);
void init_Ip_Value(void);
void FTM2_Task(void);
void sensor_init(void);

#endif /* SENSOR_CONTROL_H_ */
