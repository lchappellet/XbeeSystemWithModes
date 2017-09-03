/*
 * MotorControl.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */
/*psudocode
 * Get to a degree then measure the heading 5 times and take the EMA each time. Then go on.
 * If Horizontal motion happens(vehicle is turning) designate when to take the measurement again.
 * Maybe when we have designated the highest rssi frequency we should then try to get an accurate heading of that direction as well.
 *
 */
#include "MotorControl.h"

MotorControl::MotorControl() {
	// TODO Auto-generated constructor stub
}

MotorControl::~MotorControl() {
	// TODO Auto-generated destructor stub
}

void readEncoders() {

//Read Encoder and calculate time
//gain* encoder where gain = 360/ 1 revolution of encoder counts
//cartCurrent[0] = (.147*myEnc.read()); // gives value in degrees of where system is located for 437rpm motor
// cartCurrent[0] = (0.002667*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using OD(outerdiameters)
cartCurrent[0] = (0.002476 * myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using PD(Pitch diameters)//Best***
currentPos = cartCurrent[0];
//Serial.println("Encoders");
//Serial.println(currentPos);
//Serial.println(myEnc.read());

return;

}
Helps to calibrate the motor and home the device so encoder values start in a direction.
void CalibrateMotor180(){
	//this is usually done at the beginning when the antenna is turned on.
}
void controlpid(double setPointTarget) {

//How long since we last calculated
unsigned long timeNow = millis();
double deltaTime = (double) (timeNow - lastTimePid);

//Compute all the working error variables
double error = setPointTarget - currentPos;
errSum += (error * deltaTime);

double dErr = (error - lastErr) / deltaTime;

/*Compute PID Output*/
pidControlSignal = kp * error + ki * errSum + kd * dErr;

//Remember some variables for next time
lastErr = error;
lastTimePid = timeNow;
/*

 */
//   Serial.print(" Kp:");
//  Serial.print(kp*error);
//  Serial.print(" Ki:");
//  Serial.print(ki*errSum);
//  Serial.print(" Kd:");
//  Serial.println(kd*dErr);
return;
}

void moveWheels_PID() {
	//calculate pid = force to move
		//get encoder values.
	// calculate direction = which way to move
	// Move cart in force and direction.
	// homing
		//set home position
		//set new map of angles.
	//

if (pidControlSignal >= 0) {
	currentDir = LOW;
}
if (pidControlSignal < 0) {
	currentDir = HIGH;
}
digitalWrite(in1, !currentDir);
digitalWrite(in2, currentDir);
//  digitalWrite(in1, currentDir);       //move counterclockwise
//  digitalWrite(in2, !currentDir);

if (abs(pidControlSignal) > 120) {
	pidControlSignal = 120;
}

//Serial.println(pidControlSignal);
analogWrite(enA, abs(pidControlSignal));

//  analogWrite(enB, abs(pidControlSignal));

//  unsigned long currentMillis = millis();
//
//
//  if(currentMillis - previousMillis > interval) {
//    // save the last time you blinked the LED
//    previousMillis = currentMillis;
//     analogWrite(enA, 0);
//     analogWrite(enB, 0);
//     delay(100);
//  }

return;
}

