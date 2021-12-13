/*
 * Filter.c
 *
 *  Created on: 2021年5月17日
 *      Author: sunkaixiang
 */
#include"Filter.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>

#if Filter_type == AvgFilter
LoopArray Ip0Totle;
LoopArray Ip2Totle;
LoopArray Vref0Totle;
LoopArray Vref1Totle;
LoopArray NOxTotle;

#elif Filter_type == LowFilter
low_filter Ip0Filter;
low_filter Ip2Filter;
#elif Filter_type == DF2Filter
DF2_filter Ip0Filter;
DF2_filter Ip2Filter;
#endif




//-----------------------------------------Vref0----------------------------------------------------------
/*
const int8_t IP1_tablex[] = {-2, 0, 2, 4, 6, 8, 10, 12, 15, 20, 25, 30};//uA
const uint16_t IP0_tabley[] = {200, 500, 700, 800, 1000, 1500, 2000, 2600, 2800, 3000, 3200, 3500, 4000};//uA
const uint16_t V0_table[][13] =   {{340,340,340,340,340,340,340,340,340,340,340,350,350},//-2
								   {340,340,340,340,340,340,340,340,340,340,345,350,350},//0
								   {340,340,340,340,340,340,350,350,350,350,360,360,360},//2
								   {340,342,345,348,350,350,350,350,355,362,370,380,380},//4
								   {340,342,345,355,360,365,370,375,385,390,395,400,400},//6
								   {340,342,345,350,360,365,370,385,390,395,400,405,410},//8
								   {350,350,350,353,363,368,373,387,390,400,405,405,420},//10
								   {350,350,352,355,365,370,375,390,391,401,405,410,420},//12
								   {352,352,352,355,365,370,375,390,395,402,405,405,420},//15
								   {355,355,355,358,368,373,378,393,400,410,410,415,420},//20
								   {358,358,358,358,368,373,378,400,400,410,410,415,420},//25
								   {370,370,370,380,385,392,395,400,400,410,410,415,420}};//30uA  V0(mV)
*/
/*
const int16_t IP1_tablex[] = {-2, 0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 20, 25, 30};//uA 17
const int16_t IP0_tabley[] = {200, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000};//uA 38
const int16_t V0_table[17][38] =   {{340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 342, 345, 350, 350, 350, 350, 350, 350},//-2
								   {340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 342, 345, 345, 348, 350, 350, 350, 350, 350, 350},//0
								   {340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 340, 342, 344, 346, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 355, 360, 360, 360, 360, 360, 360, 360, 360, 360},//2
								   {340, 340, 342, 343, 345, 348, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 352, 355, 360, 362, 365, 370, 370, 375, 380, 380, 380, 380, 380, 380},//4
								   {340, 340, 342, 345, 345, 350, 350, 355, 355, 355, 355, 355, 355, 355, 360, 360, 360, 360, 360, 361, 362, 363, 364, 365, 365, 365, 365, 380, 385, 380, 370, 380, 390, 390, 390, 390, 390, 390},//5
								   {340, 342, 342, 345, 345, 355, 360, 360, 360, 365, 365, 365, 365, 365, 365, 366, 368, 370, 370, 371, 372, 373, 374, 375, 380, 385, 385, 390, 393, 395, 395, 396, 400, 400, 400, 400, 400, 400},//6
								   {340, 345, 345, 345, 350, 360, 365, 365, 365, 370, 375, 375, 375, 375, 370, 370, 370, 380, 380, 381, 382, 383, 384, 385, 390, 390, 390, 395, 395, 395, 396, 400, 403, 403, 404, 404, 404, 405},//7
								   {340, 350, 350, 350, 360, 365, 370, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 390, 391, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 405, 405, 406, 407, 408, 409, 410},//8
								   {345, 350, 350, 360, 360, 365, 370, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 390, 391, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 405, 405, 406, 407, 408, 409, 410},//9
								   {350, 350, 350, 360, 360, 365, 370, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 405, 405, 405, 405, 408, 410, 412, 414, 416},//10
								   {350, 350, 350, 360, 370, 370, 370, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 400, 405, 408, 408, 410, 412, 414, 416, 418},//11
								   {350, 350, 350, 360, 370, 370, 370, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 392, 394, 396, 398, 400, 400, 400, 400, 400, 401, 403, 405, 407, 409, 410, 412, 414, 416, 418, 420},//12
								   {351, 355, 355, 360, 370, 370, 370, 370, 373, 376, 379, 382, 385, 386, 387, 388, 389, 390, 394, 396, 398, 399, 400, 400, 400, 400, 400, 401, 403, 405, 408, 410, 410, 412, 414, 416, 418, 420},//13
								   {352, 352, 352, 360, 370, 370, 370, 370, 375, 375, 380, 385, 390, 391, 392, 393, 394, 395, 396, 396, 399, 399, 400, 400, 400, 400, 400, 402, 403, 405, 409, 410, 410, 412, 414, 416, 418, 420},//15
								   {355, 355, 355, 370, 370, 373, 373, 373, 375, 375, 380, 385, 390, 392, 394, 396, 398, 400, 398, 399, 399, 399, 400, 400, 400, 410, 410, 410, 410, 410, 410, 413, 415, 416, 417, 418, 419, 420},//20
								   {358, 358, 358, 380, 380, 380, 380, 380, 380, 382, 385, 388, 390, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 400, 410, 410, 410, 410, 410, 410, 413, 415, 416, 417, 418, 419, 420},//25
								   {370, 370, 370, 380, 380, 380, 385, 385, 385, 385, 390, 390, 392, 392, 394, 396, 398, 400, 400, 400, 400, 400, 400, 400, 400, 410, 410, 410, 410, 410, 410, 413, 415, 416, 417, 418, 419, 420}};//30uA  V0(mV)

*/

const int16_t IP1_tablex[] = {-2, 0, 2, 4, 6, 8, 10, 12, 15, 20, 25, 30};//uA 12
const int16_t IP0_tabley[] = {200,300,400,500, 600, 700, 800, 900, 1000, 3000, 3200, 3500, 4000};//uA 13
const int16_t V0_table[12][13] =   {	//2,   3,  4,  5,  6,  7,  8,  9, 10, 30, 32, 35, 40(单位：100uA)
										{340,340,340,340,340,340,350,350,350,350,350,350,350},//-2
										{340,340,340,340,340,340,350,350,350,350,350,350,350},//0
										{340,340,340,340,340,340,350,350,350,350,360,360,360},//2
										{342,342,342,342,342,345,360,360,380,380,380,380,380},//4
										{342,342,342,342,345,345,360,375,390,390,395,400,400},//6
										{342,342,342,342,345,345,360,375,400,400,400,405,410},//8
										{350,350,350,350,350,350,360,380,400,400,405,405,420},//10
										{350,350,350,350,350,352,360,380,401,401,405,410,420},//12
										{352,352,352,352,352,352,360,380,402,402,405,405,420},//15
										{355,355,355,355,355,355,360,380,410,410,410,415,420},//20
										{358,358,358,358,358,358,360,380,410,410,410,415,420},//25
										{370,370,370,370,370,370,380,380,410,410,410,415,420} //30uA  V0(mV)
																								};
const int16_t* V0_table_p[] = {V0_table[0],V0_table[1],V0_table[2],V0_table[3],V0_table[4],V0_table[5],V0_table[6],V0_table[7],V0_table[8],V0_table[9],V0_table[10],V0_table[11]};

GeneralTable IPTable =
    {
        .Xlength = sizeof(IP1_tablex)/sizeof(IP1_tablex[0]),
        .Ylength = sizeof(IP0_tabley)/sizeof(IP0_tabley[0]),
        .X_max = 0,
        .X_min = 0,
        .Y_max = 0,
        .Y_min = 0,
        .X_tableInterval = 0,
        .Y_tableInterval = 0,

        .x_table = IP1_tablex,
        .y_table = IP0_tabley,

        .Original_Table = V0_table_p,

        .New_Table = NULL,
		.new_Xlen = 0,
		.new_Ylen = 0
	};

//-----------------------------------------end----------------------------------------------------------


//-----------------------------------------Vref1----------------------------------------------------------
#if VREF1_METHOD == 1
const int8_t IP1V1_tablex[] = {6, 7, 8, 10, 15, 20};//uA
const uint16_t Vref0_tabley[] = {390, 400, 410};//mV
const uint16_t V1_table[6][3] = {{400,400,400},//6
								{400,400,400},//7
								{400,405,405},//8
								{400,410,410},//10
								{400,410,410},//15
								{400,410,410}};//20uA  V1(mV)

#endif
//-----------------------------------------end----------------------------------------------------------


//-----------------------------------------aging----------------------------------------------------------
const int16_t O2age_Widetablex[] = {4, 5, 6, 7, 8, 10, 12, 14, 18, 20, 21, 23};//%  12
const int16_t NOXage_Widetabley[] = {100, 200, 300, 500, 800, 1000, 1500, 2000, 3000};//ppm 13
const int16_t Aging_WideTable[12][9] =   { //100,200,300,500,800,1000,1500,2000,3000
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//4%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//5%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//6%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//7%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//8%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//10%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//12%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//14%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//18%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//20%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80},//21%
											{-80,-80,-80,-80,-80,-80, -80, -80, -80}//23%
																							};
const int16_t* Aging_WideTable_p[] = {Aging_WideTable[0],Aging_WideTable[1],Aging_WideTable[2],Aging_WideTable[3],Aging_WideTable[4],Aging_WideTable[5],Aging_WideTable[6],Aging_WideTable[7],Aging_WideTable[8],Aging_WideTable[9],Aging_WideTable[10],Aging_WideTable[11]};

GeneralTable AGEWideTable =
    {
        .Xlength = sizeof(O2age_Widetablex)/sizeof(O2age_Widetablex[0]),
        .Ylength = sizeof(NOXage_Widetabley)/sizeof(NOXage_Widetabley[0]),
        .X_max = 0,
        .X_min = 0,
        .Y_max = 0,
        .Y_min = 0,
        .X_tableInterval = 0,
        .Y_tableInterval = 0,

        .x_table = O2age_Widetablex,
        .y_table = NOXage_Widetabley,

        .Original_Table = Aging_WideTable_p,

        .New_Table = NULL,
		.new_Xlen = 0,
		.new_Ylen = 0
	};
//----------------------------------------------------------------------------------------------------------------------------

const int16_t O2age_Localtablex[] = {4, 5, 6, 7, 8, 10, 12, 14, 18, 20, 21, 23};//%  12
const int16_t NOXage_Localtabley[] = {0, 30, 60, 90 ,100};//ppm 13
const int16_t Aging_LocalTable[12][5] =   {  //0, 30, 60, 90,100
											{-80,-80,-80,-80,-80},//4%
											{-80,-80,-80,-80,-80},//5%
											{-80,-80,-80,-80,-80},//6%
											{-80,-80,-80,-80,-80},//7%
											{-80,-80,-80,-80,-80},//8%
											{-80,-80,-80,-80,-80},//10%
											{-80,-80,-80,-80,-80},//12%
											{-80,-80,-80,-80,-80},//14%
											{-80,-80,-80,-80,-80},//18%
											{-80,-80,-80,-80,-80},//20%
											{-80,-80,-80,-80,-80},//21%
											{-80,-80,-80,-80,-80} //23%
																		};
const int16_t* Aging_LocalTable_p[] = {Aging_LocalTable[0],Aging_LocalTable[1],Aging_LocalTable[2],Aging_LocalTable[3],Aging_LocalTable[4],Aging_LocalTable[5],Aging_LocalTable[6],Aging_LocalTable[7],Aging_LocalTable[8],Aging_LocalTable[9],Aging_LocalTable[10],Aging_LocalTable[11]};

GeneralTable AGELocalTable =
    {
        .Xlength = sizeof(O2age_Localtablex)/sizeof(O2age_Localtablex[0]),
        .Ylength = sizeof(NOXage_Localtabley)/sizeof(NOXage_Localtabley[0]),
        .X_max = 0,
        .X_min = 0,
        .Y_max = 0,
        .Y_min = 0,
        .X_tableInterval = 0,
        .Y_tableInterval = 0,

        .x_table = O2age_Localtablex,
        .y_table = NOXage_Localtabley,

        .Original_Table = Aging_LocalTable_p,

        .New_Table = NULL,
		.new_Xlen = 0,
		.new_Ylen = 0
	};
//----------------------------------------------------------------------------------------------------------------------------
const uint32_t run_timeHave[] = {0, 3000, 6000, 12000, 18000, 24000, 36000, 48000, 60000, 72000, 84000, 96000, 120000, 180000, 300000};//min
const float Aging_factor[] = {0, 0.0675, 0.125, 0.25, 0.375, 0.5, 0.667, 0.834, 1, 1, 1, 1, 1, 1, 1};//15

//-----------------------------------------end----------------------------------------------------------

float Look_Aging_factor(uint32_t runtime){
	uint16_t i;
	runtime = runtime/60;
	for(i = 0; i < sizeof(run_timeHave)/sizeof(run_timeHave[0]); i++){
		if(runtime < run_timeHave[i]){
			return Aging_factor[i - 1];
		}
	}
	return 1;
}

int myround(float a){
    return (int)(a + 0.500);
}

P_GeneralTable get_IPTable(void){
	return &IPTable;
}

P_GeneralTable get_AGELocalTable(void){
	return &AGELocalTable;
}

P_GeneralTable get_AGEWideTable(void){
	return &AGEWideTable;
}

uint16_t LinearArray_Interval(const int16_t *array){
    int16_t arrayi = array[1] - array[0];

    if(arrayi < 0){
        arrayi = (-1) *arrayi;
    }

    if(arrayi >= 1000){
        return 1000;
    }else if(arrayi >= 100){
        return 100;
    }else if(arrayi >= 10){
        return 10;
    }else{
        return 1;
    }
}


void Build_GeneralTable(GeneralTable *Table){
	uint8_t i = 0;
	float X_variable = 0, Y_variable = 0;

	float *X_bound = NULL;
	float *Y_bound = NULL;

	uint8_t y_range = 0, x_range = 0, y = 0, x = 0;
	uint8_t y_new = 0, x_new = 0;


	Y_bound = (float *)malloc(sizeof(float) * (Table->Ylength - 1));
	X_bound = (float *)malloc(sizeof(float) * (Table->Xlength - 1));
	memset(Y_bound,0,sizeof(float) * (Table->Ylength - 1));
	memset(X_bound,0,sizeof(float) * (Table->Xlength - 1));

	for(i = 0;i < Table->Ylength - 1;i++){
		Y_bound[i] = ((float)Table->y_table[i]+(float)Table->y_table[i+1])/2;

	}

	for(i = 0;i< Table->Xlength - 1;i++){
		X_bound[i] = ((float)Table->x_table[i]+(float)Table->x_table[i+1])/2;
	}

    Table->X_tableInterval = LinearArray_Interval(Table->x_table);
    Table->Y_tableInterval = LinearArray_Interval(Table->y_table);

	Table->X_min = Table->x_table[0] - Table->X_tableInterval;
	Table->X_max = Table->x_table[Table->Xlength-1] + Table->X_tableInterval;
	Table->new_Xlen = ((Table->X_max - Table->X_min)/Table->X_tableInterval) + 1;


	Table->Y_min = Table->y_table[0] - Table->Y_tableInterval;
	Table->Y_max = Table->y_table[Table->Ylength-1] +Table->Y_tableInterval;
	Table->new_Ylen = ((Table->Y_max - Table->Y_min)/Table->Y_tableInterval) + 1;


	Table->New_Table = (int16_t**)malloc(sizeof(int16_t*)*Table->new_Xlen);
	for(i = 0; i < Table->new_Xlen; i++){
		Table->New_Table[i]=(int16_t*)malloc(sizeof(int16_t)*Table->new_Ylen);
		memset(Table->New_Table[i],0,sizeof(int16_t)*Table->new_Ylen);
	}



	for(X_variable = Table->X_min;X_variable <= Table->X_max; X_variable += Table->X_tableInterval){
		x = Table->Xlength - 1;
		for(i = x_range; i < Table->Xlength-1 ; i++){
			if(X_variable < X_bound[i]){
				x = i;
				x_range = i;
				break;
			}
		}
		y_range = 0;
		for(Y_variable = Table->Y_min;Y_variable <= Table->Y_max; Y_variable += Table->Y_tableInterval){
			y = Table->Ylength - 1;
			for(i = y_range; i < Table->Ylength-1 ; i++){
				if(Y_variable < Y_bound[i]){
					y = y_range = i;
					break;
				}
			}

			Table->New_Table[x_new][y_new++] = Table->Original_Table[x][y];
			if(y_new == Table->new_Ylen) break;
		}
		y_new = 0;
		x_new += 1;
		if(x_new == Table->new_Xlen) break;
	}


    free(X_bound);
    free(Y_bound);
}

int16_t Lookup_table(float Y_data, float X_data, GeneralTable *Table){
	uint8_t x_index = 0, y_index = 0;
	if(X_data > Table->X_max){
		x_index = Table->new_Xlen - 1;
	}else if(X_data < Table->X_min){
		x_index = 0;
	}else{
		x_index = myround((X_data - Table->X_min)/Table->X_tableInterval);
	}

	if(Y_data > Table->Y_max){
		y_index = Table->new_Ylen - 1;
	}else if(Y_data < Table->Y_min){
		y_index = 0;
	}else{
		y_index = myround((Y_data - Table->Y_min)/Table->Y_tableInterval);
	}
	return Table->New_Table[x_index][y_index];


/*	uint8_t x_index = 0, y_index = 0;
	uint8_t i = 0;
	int16_t Xn = 0;
	int16_t Yn = 0;

	for(i = 0; i < Table->Xlength;i++){
		if(Table->x_table[i] > X_data){
			if(i == 0){
				Xn = Table->x_table[0];
			}else if((Table->x_table[i] - X_data) > (X_data - Table->x_table[i-1])){
				Xn = Table->x_table[i-1];
			}else{
				Xn = Table->x_table[i];
			}
			break;
		}
	}
	if(i == Table->Xlength){
		Xn = Table->x_table[Table->Xlength - 1];
	}

	for(i = 0; i < Table->Ylength;i++){
		if(Table->y_table[i] > Y_data){
			if(i == 0){
				Yn = Table->y_table[0];
			}else if((Table->y_table[i] - Y_data) > (Y_data - Table->y_table[i-1])){
				Yn = Table->y_table[i-1];
			}else{
				Yn = Table->y_table[i];
			}
			break;
		}
	}
	if(i == Table->Ylength){
		Yn = Table->y_table[Table->Ylength - 1];
	}

	if(X_data > Table->X_max){
		x_index = Table->new_Xlen - 1;
	}else if(X_data < Table->X_min){
		x_index = 0;
	}else{
		x_index = (Xn - Table->X_min)/Table->X_tableInterval;
	}

	if(Y_data > Table->Y_max){
		y_index = Table->new_Ylen - 1;
	}else if(Y_data < Table->Y_min){
		y_index = 0;
	}else{
		y_index = (Yn - Table->Y_min)/Table->Y_tableInterval;
	}
	return Table->New_Table[x_index][y_index];
	*/
}

int16_t Lookup_AGEtable(float Y_data, float X_data){
	if(Y_data < 100){
		return Lookup_table(Y_data,X_data,&AGELocalTable);
	}else{
		return Lookup_table(Y_data,X_data,&AGEWideTable);
	}
}



#define kal_Q 0.5  /*过程噪声协方差,Q增大，越相信测量值，动态响应变快，收敛稳定性变坏*/
#define kal_R 0.5  /*观测噪声协方差,R增大，越相信预测值，动态响应变慢，收敛稳定性变好*/

/*Measure 原始测量数据
  op_flg  0 仅初始化参数  1 执行卡尔曼滤波
*/
double Heat_KalmanFilter(double measure,uint8_t op_flg)
{
    double x_mid,kg,p_mid;
    static double x_last = 0,p_last = 0;
    double x_now,p_now;
    if(op_flg){         /*非初始化*/
        x_mid = x_last;
        p_mid = p_last + kal_Q;
        kg = p_mid / (p_mid + kal_R);
        x_now = x_mid + kg * (measure - x_mid);
        p_now = (1 - kg)*p_mid;

        p_last = p_now;
        x_last = x_now;
    }else{
        x_last = measure;
        p_last = kal_Q;
    }
    return x_now;
}





void Filter_init(void){

#if	VREF0_METHOD == 2
	uint8_t i = 0;
	Build_GeneralTable(&IPTable);

	Vref0Totle.maxlength = Vref0AVGTIME;
	Vref0Totle.length = Vref0AVGTIME;
	for(i = 0;i < Vref0AVGTIME; i++){
		Vref0Totle.data[i] = 0.35;
	}
	Vref0Totle.totle = 0.35*Vref0AVGTIME;
#endif

#if VREF1_METHOD == 1
	Build_V1table();
	Vref1Totle.maxlength = Vref1AVGTIME;
	Vref1Totle.length = Vref1AVGTIME;
	for(i = 0;i < Vref1AVGTIME; i++){
		Vref1Totle.data[i] = 0.405;
	}
	Vref1Totle.totle = 0.405*Vref1AVGTIME;
#endif

	Build_GeneralTable(&AGEWideTable);
	Build_GeneralTable(&AGELocalTable);

#if Filter_type == AvgFilter

	Ip0Totle.maxlength = O2CAVGTIME;
	Ip2Totle.maxlength = NOxCAVGTIME;
	NOxTotle.maxlength = NOxAVGTIME;

#elif Filter_type == LowFilter
	filter_init(&Ip0Filter, 0.12);
	filter_init(&Ip2Filter, 0.12);
#elif Filter_type == DF2Filter
	filter_init(&Ip0Filter);
	filter_init(&Ip2Filter);
#endif
}


#if Filter_type == AvgFilter
float pop(LoopArray* array)
{
	float data = 0;
	if(array->length > 0){
		data = array->data[array->head];
		array->data[array->head] = 0;
		array->head = (array->head + 1) % array->maxlength;
		array->totle -= data;
		array->length--;
	}
	return data;
}

float getValue(LoopArray* array)
{
	float data = 0;
	if(array->length > 0){
		int ptr = array->tail > 0 ? array->tail - 1 : array->maxlength - 1;
		data = array->data[ptr];
	}
	return data;
}

float push(LoopArray* array,const float data)
{
	float ret = 0;
	if(array->length < array->maxlength){
		array->data[array->tail] = data;
		array->tail = (array->tail + 1) % array->maxlength;
		array->totle += data;
		array->length ++;
	}else{
		ret = array->data[array->head];
		array->data[array->tail] = data;
		array->tail = (array->tail + 1) % array->maxlength;
		array->head = (array->head + 1) % array->maxlength;
		array->totle = array->totle + data - ret;
	}
	return ret;
}

float getavg(LoopArray* array)
{
	return ((array->length != 0)?array->totle/array->length:0);
}
#elif Filter_type == LowFilter
void filter_init(low_filter* v, float a)
{
	v->a = a;
	v->Output = 0;
}

float filter_input(low_filter* v, const float Input)
{
	float tmp;
//	int i;
	tmp = v->Output;
//	float detal = (Input - v->Output) / 10;
//	for(i = 1;i <= 10;i++){
//		tmp = v->a*(Input+detal*i) + (1 - v->a)*tmp;
//	}
	tmp = v->a*Input + (1 - v->a)*tmp;
    v->Output = tmp;
	return v->Output;
}

float filter_get(low_filter* v)
{
	return v->Output;
}
#elif Filter_type == DF2Filter
void filter_init(DF2_filter* v)
{
	v->x[0] = 0;
	v->x[1] = 0;
	v->x[2] = 0;
	v->y[0] = 0;
	v->y[1] = 0;
	v->y[2] = 0;
}
float filter_get(DF2_filter* v)
{
	return v->y[0];
}
float filter_input(DF2_filter* v, const float Input)
{
	int i;
	float detal = (Input - v->y[0]) / 10;
	for(i = 1;i <= 10;i++){
		float input = Input+detal*i;
		v->x[2] = v->x[1];v->x[1] = v->x[0];
		v->x[0] = input;
		v->y[2] = v->y[1];v->y[1] = v->y[0];
		v->y[0] = (v->x[0] + v->x[2] + 2 * v->x[1])*0.00362168 + (-0.837181651256 * v->y[2]) + (1.8226949252 * v->y[1]);
	}
	return v->y[0];
}

#endif




