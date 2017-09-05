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
#include <Arduino.h>

#include "MotorControl.h"
#include "arduino_hal.h"
#include "hal.h"
#include <avr/wdt.h>  //watchdog reset
// MotorControl::MotorControl() {
// 	// TODO Auto-generated constructor stub
//   // hal_ = dynamic_cast<IHal *> XbeeSystem::ArduinoHal(12, 15);
// }

MotorControl::MotorControl(XbeeSystem::IHal* hal, // Why does this work? why don't we just say XBeeSystem::IHal* hal_ that would skip the step of repointing at the object.
                                                  // also why don't we need hal_ to be pointing to the address of IHal objects. 
  Encoder_XBee_system* xbee_encoder,              // And how do we know that ArduinoHal is hal?  inheritance?
  int input_pin_1,
  int input_pin_2,
  int input_enable_pin)
{
  encoder = xbee_encoder;
  hal_ = hal;
  in1 = input_pin_1;
  in2 = input_pin_2;
  enA = input_enable_pin;
}


MotorControl::~MotorControl() {
	// TODO Auto-generated destructor stub
}


void MotorControl::watchdogSetup(void){
    cli();   // disable all interrupts
    wdt_reset(); // reset the WDT timer
    /*
     WDTCSR configuration:
     WDIE = 1: Interrupt Enable
     WDE = 1 :Reset Enable
     WDP3 = 0 :For 2000ms Time-out
     WDP2 = 1 :For 2000ms Time-out
     WDP1 = 1 :For 2000ms Time-out
     WDP0 = 1 :For 2000ms Time-out
    */
    // Enter Watchdog Configuration mode:
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings:
     WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    sei();
    }

//Helps to calibrate the motor and home the device so encoder values start in a direction.
void MotorControl::CalibrateMotorZeroPosition(){
  // Setting up the watchdog. This helps reset the arduino if it hasn't been called in the correct amount of time.
  //Watchdog setup *** This code helps to reset the arduino mega using a software reset in order to help zero out the encoders
       watchdogSetup();


    if(hal_->digitalReads(ZeroLocCalPin) != HIGHs){
            while(zerosystem){
                //*** tell the system watchdog to reset
                if(hal_->digitalReads(ZeroLocCalPin) == HIGHs){
                  hal_->analogWrites(enA, 0);//Stop motors
                  zerosystem = false; // sets zerosystem to false so that while loop will not start another loop.
                  delay(3000); //this gets whole arduino system to reset by not resting the watchdog timer before 2 seconds.
                  }


                else{
                    wdt_reset();
                    Serial.println("Waiting for zero Position before reseting arduino");
                    hal_->digitalWrites(in1, LOWs);
                    hal_->digitalWrites(in2, !LOWs);
                    hal_->analogWrites(enA, abs(100));//move motor at const.speed.
                    }
                }
       }
	//this is usually done at the beginning when the antenna is turned on.

}

double MotorControl::controlpid(double targetDestination){

  Serial.println("About to read encoder:");
 double encoderValue = encoder->read_encoder();
  Serial.println(encoderValue);
 double  pidControlValue= 0;

 //How long since we last calculated
 unsigned long timeNow = hal_->timemillis();
 Serial.println("Got time now");
 double deltaTime = (double)(timeNow - previousTime);
 //Compute all the working error variables
  Serial.println("Calculating error...");
 double error = targetDestination - encoderValue;
 errSum += (error * deltaTime);
  Serial.println("Calculating dErr error...");

double dErr = (error - lastErr) / deltaTime;

 /*Compute PID Output*/
 pidControlValue = kp * error + ki * errSum + kd * dErr;
 Serial.print("Pid control");
 Serial.print(pidControlValue);

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
  currentDir = 0;
  // currentDir = LOW;
}
if(pidControlvalue1<0){
  currentDir = 1;
  // currentDir = HIGH;

}
hal_->digitalWrites(in1, !currentDir);
hal_->digitalWrites(in2, currentDir);
//  digitalWrite(in1, currentDir);       //move counterclockwise
//  digitalWrite(in2, !currentDir);


if(abs(pidControlvalue1) > 120){
  pidControlvalue1 = 120;

}

//Serial.println(pidControlSignal);
hal_->analogWrites(enA, abs(pidControlvalue1));

return;
}

