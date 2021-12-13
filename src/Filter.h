/*
 * Filter.h
 *
 *  Created on: 2021Äê5ÔÂ17ÈÕ
 *      Author: sunkaixiang
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>

#define ArrayOffset		7
#define ArrayMaxLength	128
typedef struct{
	float data[ArrayMaxLength];
	uint16_t maxlength;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
	float totle;
}LoopArray;
typedef struct
{
     float Output;
     float a;
}low_filter;
typedef struct
{
	float x[3];
	float y[3];
}DF2_filter;

typedef struct{
	uint8_t Xlength;
	uint8_t Ylength;
    int16_t X_max;
    int16_t X_min;
    int16_t Y_max;
    int16_t Y_min;
    uint16_t X_tableInterval;
    uint16_t Y_tableInterval;

    const int16_t *x_table;
	const int16_t *y_table;

    const int16_t **Original_Table;

    int16_t **New_Table;
	uint8_t new_Xlen;
	uint8_t new_Ylen;
}GeneralTable,*P_GeneralTable;

#if Filter_type == AvgFilter
float pop(LoopArray* array);
float push(LoopArray* array,const float data);
float getavg(LoopArray* array);
float getValue(LoopArray* array);

#elif Filter_type == LowFilter
void filter_init(low_filter* array, float a);
float filter_input(low_filter* array, const float Value);
float filter_get(low_filter* array);
#elif Filter_type == DF2Filter
void filter_init(DF2_filter* v);
float filter_input(DF2_filter* array, const float Value);
float filter_get(DF2_filter* array);
#endif
void Filter_init(void);

float Look_Aging_factor(uint32_t run_time);
P_GeneralTable get_AGEWideTable(void);
P_GeneralTable get_AGELocalTable(void);
int16_t Lookup_AGEtable(float Y_data, float X_data);
P_GeneralTable get_IPTable(void);
int16_t Lookup_table(float Y_data, float X_data, GeneralTable *Table);
double Heat_KalmanFilter(double measure,uint8_t op_flg);

#endif /* FILTER_H_ */
