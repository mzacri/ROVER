/*****************************************************************************//**
  * @file    lowPassFilters.c
  * @author  M'barek ZACRI
  * @date    25 mai 2020
  * @brief 	 Numeric low pass filters
 *******************************************************************************/
#include "math.h"

//Filtering rearSpeed low pass
float LowPassFilter1(float speed){
	static float so=0,so0=0,sc=0,sc0=0;
	so0=so;
	sc0=sc;
	sc=speed;
	so=(-0.47619*(sc+sc0)-so0)/-1.952381;//(b1*(sc+sc0)-so0)/a1 with a1=(Te+2Tau)/(Te-2Tau) and b1=K*Te/(Te-2Tau);
	return so;
}

//Filtering servoPose :
float LowPassFilter2(float servoPose){
	static float s=0,s0=0,s1=0,s2=0,a0=0,a1=0,a2=0;
	//s=0.003021139*(servoPose+3*a2+3*a1+a0+267.003*s2+326.997*s1-270.999*s0);//100ms bandwidth//1/alpha*(servoPose+3*a2+3*a1+a0-beta*s2-gamma*s1-xsi*s0);
	s=0.0109890*(servoPose+3*a2+3*a1+a0+57.000375*s2+86.999625*s1-60.999875*s0);//1/alpha*(servoPose+3*a2+3*a1+a0-beta*s2-gamma*s1-xsi*s0);
	s0=s1;
	s1=s2;
	s2=s;
	a0=a1;
	a1=a2;
	a2=servoPose;
	return s;
}
//Filtering battery Voltage :
float LowPassFilter3(float batteryVolt){
	static float so=0,so0=0,sc=0,sc0=0;
	so0=so;
	sc0=sc;
	sc=batteryVolt;
	so=(-0.071428*(sc+sc0)-so0)/-1.142857;//(b1*(sc+sc0)-so0)/a1 with a1=(Te+2Tau)/(Te-2Tau) and b1=K*Te/(Te-2Tau);
	return so;
}
