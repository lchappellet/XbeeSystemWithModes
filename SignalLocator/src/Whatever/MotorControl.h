/*
 * MotorControl.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "hal.h"

class MotorControl {
public:
    MotorControl();
    virtual ~MotorControl();

    // arduino setup pins
	  //motor control variables
	    const int enA = 12;
	    const int in1 = 11;
	    const int in2 = 9;

    XbeeSystem::IHal* hal_;


	 	// Location to move to
	 	double targetDestination =0;


	    //Current Position
	    double encoderValue = 0;

	    //Encoder position variable
	    long encoderPosPrev  = -999;


	    //Time Variable
	   	double previousTime = 0;


	    //Control Variables
	    double kp = 10;
	    double ki = 0;
	    double kd = 40;
	    double lastErr = 0;
	    double errSum = 0;
	    double pidControlValue = 0;

	    bool currentDir;

	    float cartCurrent[3] = {0,0,0};


	    // functions
	    double controlpid(double encoderValue, double targetDestination);
	    void move_motor(double pidControlvalue1);
	    void CalibrateMotor180();
};

#endif /* MOTORCONTROL_H_ */
