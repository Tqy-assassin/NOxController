/*
 * Survey_Control.h
 *
 *  Created on: 2019Äê7ÔÂ25ÈÕ
 *      Author: tianqingyuan
 */

#ifndef SURVEY_CONTROL_H_
#define SURVEY_CONTROL_H_

#include "stdio.h"
#include "derivative.h"

enum{
	N =	7,
	NB = -3,
	NM = -2,
	NS = -1,
	ZO = 0,
	PS = 1,
	PM = 2,
	PB = 3,
}FuzzyField;
static const int KpMatrix[7][7] = {	{ZO,ZO,NM,NM,NM,NB,NB},
									{PS,ZO,NS,NM,NM,NM,NB},
									{PS,PS,ZO,NS,NS,NM,NM},
									{PM,PM,PS,ZO,NS,NM,NM},
									{PM,PM,PM,PS,ZO,NS,NS},
									{PB,PB,PM,PS,PS,ZO,NS},
									{PB,PB,PM,PM,PS,ZO,ZO}};

static const int KiMatrix[7][7] = {	{ZO,ZO,PS,PM,PM,PB,PB},
									{ZO,ZO,PS,PS,PM,PB,PB},
									{NM,NS,ZO,PS,PS,PM,PB},
									{NM,NM,NS,ZO,PS,PM,PM},
									{NB,NM,NS,NS,ZO,PS,PS},
									{NB,NB,NM,NS,NS,ZO,ZO},
									{NB,NB,NM,NM,NS,ZO,ZO}};

static const int KdMatrix[7][7] = {	{PB,PM,PM,PM,PS,PS,PB},
									{PB,PS,PS,PS,PS,PS,PB},
									{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
									{ZO,NS,NS,NS,NS,NS,ZO},
									{ZO,NS,NM,NM,NS,NS,ZO},
									{PS,NS,NB,NM,NM,NS,ZO},
									{PS,NS,NB,NB,NB,NM,PS}};

static const int trimf_para[N*3] ={	NB,NB,NM,	//Trigonometric membership function parameters
									NB,NM,NS,
									NM,NS,ZO,
									NS,ZO,PS,
									ZO,PS,PM,
									PS,PM,PB,
									PM,PB,PB};

#define IP0EMax		(3)
#define IP0dEmax	(3)
#define IP0KpMax	(0.005)
#define IP0KiMax	(0.00005)
#define IP0KiMin	(0.000001)
#define IP0KdMax	(0.005)
#define IP0dKpMax	(0.0001)
#define IP0dKiMax	(0.00001)
#define IP0dKdMax	(0.0001)

typedef struct{
	float E;
	float E_1;
	float E_2;
	float Kp;
	float Kd;
	float Ki;
}SurveyVariate_Type;

typedef struct{
	float E;
	float E_1;
	float E_2;
	float dE;
	float Kp;
	float Kd;
	float Ki;
	float Ke;      				//Ke=n/emax,	Field:[-3,-2,-1,0,1,2,3]
	float Kde;     				//Kde=n/demax, 	Field:[-3,-2,-1,0,1,2,3]
	float Ku_p;   				//Ku_p=Kpmax/n,	Field:[-3,-2,-1,0,1,2,3]
	float Ku_i;    				//Ku_i=Kimax/n,	Field:[-3,-2,-1,0,1,2,3]
	float Ku_d;    				//Ku_d=Kdmax/n,	Field:[-3,-2,-1,0,1,2,3]
}SurveyVariate_extern_Type;
void fuzzy_Controller(SurveyVariate_extern_Type* p);
float trimFuzzification(float x,int a,int b,int c);
#endif /* SURVEY_CONTROL_H_ */
