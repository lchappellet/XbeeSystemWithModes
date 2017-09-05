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
#include <avr/wdt.h> //watchdog reset
class MotorControl {
public:
    MotorControl(XbeeSystem::IHal* hal,
      Encoder_XBee_system* encoder, // should we use a different name than just encoder here?
      int input_pin_1,
      int input_pin_2,
      int input_enable_pin);

    virtual ~MotorControl();

    // arduino setup pins
	  //motor control variables
	    int enA = 12;
	    int in1 = 11;
	    int in2 = 9;
	    //creating interface for arduino object and the encoder object
        XbeeSystem::IHal* hal_;  
        Encoder_XBee_system* encoder; // Is this redundant? or is this creating the object. 


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


		//Watchdog and Zero location Calirbration variables
		const int ZeroLocCalPin = 4; // pin to read when the zero cal limit switch goes off
		bool zerosystem = true;// true,false for the while loop in setup
		const int HIGHs = 1; //HIGH and LOW stand for 0 and 1 which help to turn on the pins or not.
		const int LOWs = 0;
		    // Variables to be created for CalibrateMotor180
		    //int CalZeroPosition = 0; // variable for the zero calibration encoder reading
		    //int CalOneEightyPosition = 0;//Varibale to hold 180 encoder reading
		   
		     //  const int OneEightyCalPin = 5;//Pin to read when 180 limit switch goes off . NOT USING THIS yet
		     // bool calibFinished = true;
		     // int TotalEncoderCalDegrees  = 0; //keeps the total number of encodervalues
		     //  int ActivatedOneEighty = 0; //tells us that the 180 degree limit has been switched.
		    

	    // functions
	    double controlpid(double targetDestination);
	    void move_motor(double pidControlvalue1);
	    void CalibrateMotorZeroPosition();   // use to be called void CalibrateMotor180();
	    void watchdogSetup(void);
};

#endif /* MOTORCONTROL_H_ */
