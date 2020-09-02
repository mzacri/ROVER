/*****************************************************************************//**
  * @file    CompFilter.c
  * @author  M'barek ZACRI
  * @date    1 mai 2020
  * @brief 	Complementary filtering to compute Eulers angles by fusing
  * accelerometer's and gyroscope's noisy data.
 *******************************************************************************/
#include <Sensors/Filters/EulerOrientationCompFilter.h>
#include <Sensors/MPU6050/sd_hal_mpu6050.h>
#include "math.h"
#define PI              3.141592653f            /*!< PI definition */
#define G				9.81
#define sind(x) (sin(fmod((x),360) * PI / 180))
#define cosd(x) (cos(fmod((x),360) * PI / 180))
#define tand(x) (tan(fmod((x),360) * PI / 180))
#define RAD2DEG(x)      ((x) * 57.2957795f)     /*!< Radians to degrees converter */
#define POW2(x)			(x*x)

void computeAccelAngles(EULER_ANGLES *angles,SD_MPU6050 mpu){
	angles->accel_roll = RAD2DEG(atan(mpu.Accelerometer_Y/sqrt(POW2(mpu.Accelerometer_Z)+POW2(mpu.Accelerometer_X))));
	angles->accel_pitch = RAD2DEG(atan(-mpu.Accelerometer_X/sqrt(POW2(mpu.Accelerometer_Z)+POW2(mpu.Accelerometer_Y))));
}

void computeGyroAngles(EULER_ANGLES *angles,SD_MPU6050 mpu,float deltaT){
	if(angles->pitch != 90){
		angles->gyro_roll = angles->roll+deltaT*(mpu.Gyro_Mult*(mpu.Gyroscope_X+mpu.Gyroscope_Y*sind(angles->roll)*tand(angles->pitch)+mpu.Gyroscope_Z*cosd(angles->roll)*tand(angles->pitch)));
		angles->gyro_pitch = angles->pitch+deltaT*(mpu.Gyro_Mult*(mpu.Gyroscope_Y*cosd(angles->roll)-mpu.Gyroscope_Z*sind(angles->roll)));
		angles->gyro_yaw = angles->yaw+deltaT*(mpu.Gyro_Mult*(mpu.Gyroscope_Y*sind(angles->roll)/cosd(angles->pitch)+mpu.Gyroscope_Z*cosd(angles->roll)/cosd(angles->pitch)));
	}
}
void computeAngles(EULER_ANGLES *angles,float alpha){
	angles->roll = (1-alpha)*angles->gyro_roll+alpha*angles->accel_roll;//(1-alpha)*angles->gyro_roll+alpha*angles->accel_roll;
	angles->pitch =(1-alpha)*angles->gyro_pitch+alpha*angles->accel_pitch;//(1-alpha)*angles->gyro_pitch;+alpha*angles->accel_pitch;
	angles->yaw = angles->gyro_yaw;
}

void initAngles(EULER_ANGLES *angles,SD_MPU6050 mpu){
	computeAccelAngles(angles,mpu);
	angles->gyro_roll = angles->accel_roll;
	angles->gyro_pitch = angles->accel_pitch;
}


