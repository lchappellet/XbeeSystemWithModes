/*
 * MotorControl.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "hal.h"
// #include "Encoder.h"
#include "EncoderXbeeSystem.h"

class MotorControl {
public:
    MotorControl(XbeeSystem::IHal* hal,
      Encoder_XBee_system* encoder,
      int input_pin_1,
      int input_pin_2,
      int input_enable_pin);

    virtual ~MotorControl();

    // arduino setup pins
	  //motor control variables
	    int enA = 12;
	    int in1 = 11;
	    int in2 = 9;
      XbeeSystem::IHal* hal_;
      Encoder_XBee_system* encoder;


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
	    double controlpid(double targetDestination);
	    void move_motor(double pidControlvalue1);
	    void CalibrateMotor180();
};

#endif /* MOTORCONTROL_H_ */
