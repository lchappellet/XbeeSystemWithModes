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
#include "arduino_hal.h"
#include "hal.h"
// #include <Arduino.h>

MotorControl::MotorControl() {
	// TODO Auto-generated constructor stub
  hal_ = <IHal*> XbeeSystem::ArduinoHal(12, 15);
}

MotorControl::~MotorControl() {
	// TODO Auto-generated destructor stub
}


//Helps to calibrate the motor and home the device so encoder values start in a direction.
void CalibrateMotor180(){
	//this is usually done at the beginning when the antenna is turned on.
}

double MotorControl::controlpid(double encoderValue, double targetDestination){
//  pidControlValue
 double  pidControlValue= 0;

 //How long since we last calculated
 unsigned long timeNow = millis();
 double deltaTime = (double)(timeNow - previousTime);

 //Compute all the working error variables
 double error = targetDestination - encoderValue;
 errSum += (error * deltaTime);

double dErr = (error - lastErr) / deltaTime;

 /*Compute PID Output*/
 pidControlValue = kp * error + ki * errSum + kd * dErr;

 //Remember some variables for next time
 lastErr = error;
 previousTime = timeNow;

//   Serial.print(" Kp:");
//  Serial.print(kp*error);
//  Serial.print(" Ki:");
//  Serial.print(ki*errSum);
//  Serial.print(" Kd:");
//  Serial.println(kd*dErr);
return  pidControlValue;
}


void MotorControl::move_motor(double pidControlvalue1){

if(pidControlvalue1>=0){
  currentDir = LOW;
}
if(pidControlvalue1<0){
  currentDir = HIGH;

}
hal_.digitalWrite(in1, !currentDir);
hal_.digitalWrite(in2, currentDir);
//  digitalWrite(in1, currentDir);       //move counterclockwise
//  digitalWrite(in2, !currentDir);


if(abs(pidControlvalue1) > 120){
  pidControlvalue1 = 120;

}

//Serial.println(pidControlSignal);
hal_.analogWrite(enA, abs(pidControlvalue1));

return;
}

