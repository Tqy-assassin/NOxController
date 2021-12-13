/*
 * Survey_Control.c
 *
 *  Created on: 2019Äê7ÔÂ25ÈÕ
 *      Author: tianqingyuan
 */

#include "Survey_Control.h"

float trimFuzzification(float x,int a,int b,int c){
	float temp;
	x = x < a ? a : (x > c ? c : x);

	if(x<b){
		if(a == b){
			temp = ((float)c-x)/((float)c-(float)b);
		}else{
			temp = (x-(float)a)/((float)b-(float)a);
		}
	}else{
		if(b == c){
			temp = (x-(float)a)/((float)b-(float)a);
		}else{
			temp = ((float)c-x)/((float)c-(float)b);
		}
	}
	return temp;
}

void fuzzy_Controller(SurveyVariate_extern_Type* p)
{
	float u_e[N],u_de[N];
	int u_e_index[2],u_de_index[2];				//Stores the index of the actived
//	float delta_Kp;
	float delta_Ki;
//	float delta_Kd;
	float e,de;

	float den=0,num=0;
	int m,n;
	int i,j=0;

	e = p->E/p->Ke;
	de = p->dE/p->Kde;

	for(i=0,j=0;i<N;i++)	//fuzzy up e
	{
		u_e[i]=trimFuzzification(e,trimf_para[i*3]*1000,trimf_para[i*3+1]*1000,trimf_para[i*3+2]*1000);		//fuzzification
		if(u_e[i]!=0)
			u_e_index[j++]=i;                //Stores the index of the actived
	}
	for(;j<2;j++)
		u_e_index[j]=0;

	for(i=0,j=0;i<N;i++)		//fuzzy up de
	{
		u_de[i]=trimFuzzification(de,trimf_para[i*3]*1000,trimf_para[i*3+1]*1000,trimf_para[i*3+2]*1000);	//fuzzification
		if(u_de[i]!=0)
	    	u_de_index[j++]=i;            	//Stores the index of the actived
	}
	for(;j<2;j++)
		u_de_index[j]=0;

//===============================calculate Kp=========================================
//	den = 0,num = 0;
//	for(m = 0;m < 2;m++){
//		for(n=0;n<2;n++)
//		{
//			num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*KpMatrix[u_e_index[m]][u_de_index[n]];
//			den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
//		}
//	}
//	if(den <= 1)
//	{
//		delta_Kp = 0;
//	}else{
//		delta_Kp = num/den;
//	}
//	delta_Kp*=p->Ku_p;
//	p->Kp+=delta_Kp;
//	if(p->Kp>=IP0KpMax){
//		p->Kp = IP0KpMax;
//	}
//	else if(p->Kp < 0){
//		p->Kp = 0;
//	}
//====================================================================================
//===============================calculate Ki=========================================
	den = 0,num = 0;
	for(m = 0;m < 2;m++){
		for(n=0;n<2;n++)
		{
			num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*KiMatrix[u_e_index[m]][u_de_index[n]];
			den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	}
	if(den <= 1)
	{
		delta_Ki = 0;
	}else{
		delta_Ki = num/den;
	}
	delta_Ki*=p->Ku_i;
	p->Ki+=delta_Ki;
	if(p->Ki>=IP0KiMax){
		p->Ki = IP0KiMax;
	}
	else if(p->Ki < IP0KiMin){
		p->Ki = IP0KiMin;
	}
//====================================================================================
//===============================calculate Kd=========================================
//	den = 0,num = 0;
//	for(m = 0;m < 2;m++){
//		for(n=0;n<2;n++)
//		{
//			num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*KdMatrix[u_e_index[m]][u_de_index[n]];
//			den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
//		}
//	}
//	if(den <= 1)
//	{
//		delta_Kd = 0;
//	}else{
//		delta_Kd = num/den;
//	}
//	delta_Kd*=p->Ku_d;
//	p->Kd+=delta_Kd;
//	if(p->Kd>=IP0KdMax){
//		p->Kd = IP0KdMax;
//	}
//	else if(p->Kd < 0){
//		p->Kd = 0;
//	}
//====================================================================================
}
