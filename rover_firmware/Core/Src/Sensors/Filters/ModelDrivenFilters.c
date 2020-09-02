/*
 * ModelDrivenFilters.c
 *
 *  Created on: 6 juil. 2020
 *      Author: mbarek.zacri
 */
#include <math.h>
#include "RoverConfig.h"
//Rear speed model driven filter:
float SpeedFilter(float speed,float brushlessControl){
	static float so=0,so0=0,sc=0,sc0=0, speedFiltered=0;
	float in=brushlessControl;
	if(in>150){
		sc=0.006964*in-1.0022;
		//sc=0.009392857*in-1.39642857;
	} else if (in<-160){
		sc=0.010706*in+1.6592;
		//sc=0.016*in+2.81142857;
	} else {
		sc=0;
	}
	so=(-0.033333*(sc+sc0)-so0)/-1.066667;
	//so=(-0.02564102*(sc+sc0)-so0)/-1.05128205;
	sc0=sc;
	so0=so;
	if (fabs(speed)<MINSPEED){
	    	speedFiltered=0;
	} else if( fabs(speed-so)<0.6 && brushlessControl >0){   //Fourchette de comparaison
    	speedFiltered=speed;
    } else if( fabs(speed-so)<0.2 && brushlessControl <0){
    	speedFiltered=speed;
    }
	return speedFiltered;
}

//Servo pose model driven filter:
float ServoPoseFilter(float servoPose,float servoControl){
		static float so=0,so0=0,sc0=0,sc=0,ret=0;
		sc=-servoControl;
		so=(-5.1198257e-5 * (sc+sc0) - so0)/-1.2222;
		sc0=sc;
		so0=so;
        if(fabs(servoPose)<0.1 || fabs(servoPose-so)<=0.9*fabs(so)){
			ret=servoPose;
	    }
	    return ret;
}
