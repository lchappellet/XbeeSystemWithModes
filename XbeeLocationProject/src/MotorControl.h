/*
 * MotorControl.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

class MotorControl {
public:

	  //motor control variables
	    const int enA = 12;
	    const int in1 = 11;
	    const int in2 = 9;

	    //Encoder Object
	 	    Encoder myEnc(18, 19);






	    //Time Variables
	    unsigned long lastTimePid;



	    //Current Position
	    double currentPos = 0;

	    //Encoder position variable
	    long encoderPosPrev  = -999;

	    //Control Variables
	    double kp = 10;
	    double ki = 0;
	    double kd = 40;
	    double lastErr = 0;
	    double errSum = 0;
	    double pidControlSignal = 0;
	    double zero = 0;        // helps with tracking setpoint value



	    bool currentDir;
	    bool before = true;

	    //Target Set Point Variable
	    const double firstValue = 160;
	    double setPointTarget = firstValue;
	    double setPointTargetValue = firstValue;
	    double setPointTargetValue1 = firstValue;
	    double motorAngle = 0;
	    double interval = 4000;                   // amount of time millis is waiting to then turn off motors in moveWheels() function
	    double interval1 = 4000;
	    long previousMillis = 0;



	    //Feed Forward Variables
	    //float cartPrev[2] = {0,0};
	    float cartCurrent[3] = {0,0,0};

	    // Variables to be created for CalibrateMotor180
	    int CalZeroPosition = 0; // variable for the zero calibration encoder reading
	    int CalOneEightyPosition = 0;//Varibale to hold 180 encoder reading
	    const int ZeroCalPin = 4; // pin to read when the zero cal limit switch goes off
	    const int OneEightyCalPin = 5;//Pin to read when 180 limit switch goes off
	    bool calibFinished = true;
	    int TotalEncoderCalDegrees  = 0; //keeps the total number of encodervalues
	    int ActivatedOneEighty = 0; //tells us that the 180 degree limit has been switched.
	    bool zerosystem = true;// true,false for the while loop in setup



	MotorControl();
	virtual ~MotorControl();
};

#endif /* MOTORCONTROL_H_ */
