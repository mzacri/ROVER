/*****************************************************************************//**
  * @file    speedControl.c
  * @author  M'barek ZACRI
  * @date    27 avr. 2020
  * @brief 	 Related definitions to the control of the rover actuators.
 *******************************************************************************/

#ifndef SRC_SPEEDCONTROL_C_
#define SRC_SPEEDCONTROL_C_

#include <RoverConfig.h>
#include "main.h"
#include <string.h>
#include <Sensors/RoverStateSensors.h>
//Internal command alternator
void intCommandAlternator(float *command, float* servoSpeed){
//Command execution:
	 static int k=0;
	 if(k == 0){
		 *command=COMMAND;
		 //*servoSpeed=1;
		 k=3;
	 } else if (k==1 || k == 3) {
		 *command=0;
		 //*servoSpeed=0;
		 k=k-1;
	 } else {
		 *command=-COMMAND;
		 //*servoSpeed=-1;
		 k=1;
	 }
}

// --------State functions:

//Compute Servo position:
int Servo_command(float servoCommand){
	int servoControl=0;
	servoControl=(int)(servoCommand*1536.170213);  //servoCommand == position in rad
	if(servoControl>722){
		servoControl=722;
	} else if (servoControl<-722){
		servoControl=-722;
	}
	return servoControl;
}


//-------------------Control functions:

///****RE:
#if CONTROL == RE
int Re(float re_command, float re_speed){
	//Control:
		static float xr,e,re_control=0;
		if(re_command==0){
			re_control=0;
			e=0;
			xr=0;
			re_control=0;
		} else if (re_command>0) {
			if(re_command<0.2){
				re_command=0.2;
			}
			e=re_command-re_speed;
			xr+=e;
			re_control= 160.6297*re_command+3.6708*xr- 54.1658*re_speed+MIN_POSITIVE_COMMAND; //g*re_command-S(1)*xr+S(0)*re_speed
			if(re_control>1500){
				re_control=1500;
			}

		} else if (re_command<0) {
			if(re_command>-0.4){
				re_command=-0.4;
			}
			e=re_command-re_speed;
			xr+=e;
			re_control=  94.0459*re_command + 1.1726*xr - 31.5459 *re_speed + MIN_NEGATIVE_COMMAND;
			if(re_control<-1500){
				re_control=-1500;
			}
		}
		return (int)re_control;
}
//OpenLoop:
#elif CONTROL == OL
int OpenLoop(float ol_command){
	static float ol_control=0;
	static int cnt=0;
	if(ol_command>0){
		//ol_control=(ol_command+1.0022)/0.006964;
		ol_control=(ol_command+1.4)/0.009393;
		if(ol_control<MIN_POSITIVE_COMMAND ){
			ol_control=MIN_POSITIVE_COMMAND ;
		}
	} else if(ol_command<0){
		//ol_control=(ol_command-1.6592)/0.010706;
		ol_control=(ol_command-2.81)/0.016;
		if (cnt<CNT_PUNCH){
			ol_control=-PUNCH;
			cnt++;
		} else if(ol_control>MIN_NEGATIVE_COMMAND){
			ol_control=MIN_NEGATIVE_COMMAND;
		}
	} else {
		ol_control=0;
		cnt=0;
	}
	if(ol_control<-1500){
		ol_control=-1500;
	} else if (ol_control > 1500) {
		ol_control= 1500;
	}
	return ol_control;
}
#endif

#endif /* SRC_SPEEDCONFIG_C_ */
