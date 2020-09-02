/**
 *
 ******************************************************************************
  * @file    RoverConfig.h
  * @author  M'barek ZACRI
  * @date    27 avr. 2020
  * @brief 	 Rover configuration file. Sensors tuning, control configuration
  *  and different parameters definitions.
 ******************************************************************************

 */

#ifndef INC_CONFIGFILES_ROVERCONFIG_H_
#define INC_CONFIGFILES_ROVERCONFIG_H_

#include <math.h>

//Modes panel:
#define CONTROL								RE   //OL: openloop; RE;
#define COMMAND_SOURCE						RASP  //Source of command: INT for internal or RASP for raspberry

#define COMMAND								0.4   //Speed value ( m/s ) if COMMAND_SOURCE is INT: command to execute ( cf intCommandExecute() )
#define MAX_COMMAND							1.2   //Safety maximum speed to command. If out of bound, rover is stopped.

//Control:
#define BATTERY_VOLTAGE_THRESHOLD			4  //Battery voltage threshold, if voltage < threshold: led2 stop blinking

//Tuning of brushless motor speed sensor:
#define SENSOR								4     //3-XOR pulse width with an estimation of direction //4-XOR pulse width with a calculation of direction
#define MINSPEED							0.05    //Minimal detectable speed m/s
#define MAXSPEED							5      //Maximal detectable speed m/s -> speed filtring from rotor vibrations errors
#define REDUCTION_RATIO						12   //Transmission Gear ratio
#define WHEELS_RADIUS						0.06   //If changed, recalculate RE control


//Brushless motor dead zone management:
#define MIN_POSITIVE_COMMAND 				150 //170   //Open loop minimal command values to start rotating the brushless motor:
#define MIN_NEGATIVE_COMMAND				-195 //-225

#define DEAD_ZONE							1   //Use (1) or not (0) the dead zone management
#define PUNCH								250   // First punch to avoid brushless dropout in openloop reverse
#define CNT_PUNCH							2



//Tuning of Servo motor:
#define SERVO_ZERO_POSE						5300//5265  //When changed, servo pose sensor should be re-calibrated
#define ADJUST_POSITIVE						0   //Adjust SERVO_ZERO_POSE according to the servoPosition and the command
#define ADJUST_NEGATIVE						0  //Adjust SERVO_ZERO_POSE according to the servoPosition and the command
#define ESTIMATE_SERVO_POSE					1 	//If 1, servoPose is estimated instead of filtering the ADC Output. In practice estimation have nicer odometry results, choice is yours.

//FreeRTOS:
//#define SYSTEMVIEW   //Uncomment if you want to use SystemView ( cf. HowTo_SystemView.txt )



//Modes definitions: (Do not modify)
#define OL									1   	//Open loop -> Identification
#define RE									3


#define INT									0
#define RASP								1


#define MINTIMESTAMP 						(int)(((float)20/RATIO)*((float)HAL_RCC_GetHCLKFreq()/((TIM5->PSC+1)*MAXSPEED)))
#define MAXTIMESTAMP						(int)(((float)20/RATIO)*((float)HAL_RCC_GetHCLKFreq()/((TIM5->PSC+1)*MINSPEED)))
#define RATIO								(float)(9.5487179*REDUCTION_RATIO/WHEELS_RADIUS)  //rear wheels m/s to motor axe tr/min
#endif /* INC_CONFIGFILES_ROVERCONFIG_H_ */
