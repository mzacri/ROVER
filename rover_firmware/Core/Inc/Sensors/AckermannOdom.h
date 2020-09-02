/*****************************************************************************//**
  * @file    AckermannOdom.h
  * @author  M'barek ZACRI
  * @date    6 juil. 2020
  * @brief 	 Compute odometry using ackermann steering model
 *******************************************************************************/

#ifndef ACKERMANNODOM_H
#define ACKERMANNODOM_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <Sensors/RoverStateSensors.h>

extern void AckermannOdom(double v, double phi, float speedVar, float steeringVar,
        float *w, float M[4]);
extern void AckermannOdom_init(void);
extern void Ackermann_RetrieveOdom(ROVER_STATE* state);

#endif

