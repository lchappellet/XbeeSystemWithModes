//antenna file is the antenna object to control how the antenna moves.
// AntennaControl.h
#ifndef AntennaControl_H_
#define AntennaControl_H_

#include "EncoderXBeesystem.h"
#include "MotorControl.h"
//these two includes below are so I can use arduino functions in c++ and still have my code compile
//This does mean that I need them to call serial() digitalWrite() and others
#include "arduino_hal.h"
#include "hal.h"

// In this file I would like to create an antennaControl
// object which controls the antenna and tells it where I want it to move

//1. In the end I want the antenna to do a sweep motion so that it can get a scan of the field

// 2. Then I also want the ability for the antenna to focus in on an object and do smaller sweeps
// around the highest frequency of the signal.

// 3. If the signal starts getting weeker than the highest frequency. The antenna should
// start to do larger and larger sweeps until it hones in on the antenna.
// (the last motion could have issues when you get farther away but that
// useful to tell people signal strength is decreasing. )

// 4. Antenna need to read in values

// 5. Need to home the antenna by using an accurate

// 1. goal: move the antenna in big 360 degree sweeps
//
//        -Send the antenna to the zero location.
//        -Then Send the antenna to the 360 degree location.
//        -
class Antenna_Control{

public:
  Antenna_Control();
	virtual ~Antenna_Control();

public:
  //set the object motorcontrol so we can call on the class to find the pid and move the motors
  MotorControl motorcontrol;
  //set the object encoder so that we can know our current location of the encoder
  // This will help us to know we have reached close enough to our destination.
  Encoder_XBee_system encoder;
// create the arduino object below so I can use serial and other functions
//  ArduinoHal arduino;

  void moveTo360Degrees();
  void moveToZeroDegrees();
  void honeInRight();
  void honeInLeft();


  int zero = 0;   // we need to CHANGE this value
  int threeFourty = 340;




};





















#endif /*AntennaControl_H_*/
