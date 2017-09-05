// MOTOR CONTROL ARDUINO
//Whats up team
// #include "hal.h"
#include <Arduino.h>
#include "arduino_hal.h"
#include "MotorControl.h"
#include <Encoder.h>
#include "EncoderXBeeSystem.h"
#include <avr/wdt.h> //watchdog reset
using namespace XbeeSystem;

const int DIG_IN_PIN = 11;
const int DIG_IN_2_PIN = 9;
const int INPUT_ENABLE_PIN = 12;
const int DESIRED_TARGET_DESTINATION = -80;

ArduinoHal hal;
Encoder encoder(18, 19);
Encoder_XBee_system xbee_encoder(&encoder);

MotorControl motor(&hal, &xbee_encoder, DIG_IN_PIN, DIG_IN_2_PIN, INPUT_ENABLE_PIN);

double lastEncoder = 0;
double lastPid = 0;





void setup() {
  Serial.begin(9600);
  Serial.println("Setup done");
  double vall = xbee_encoder.read_encoder();
  Serial.println("Read encoder!");
  Serial.println(vall);
  motor.CalibrateMotorZeroPosition(); //This will calibrate the motor to a zero position. 
                                      /*Once at the zero position this will not activate 
                                        on the second time the arduino turns on do to the
                                       if statement being false before the while loop. */
}

void loop() {
  // put your main code here, to run repeatedly:
  // lastEncoder = xbee_encoder.read_encoder();
  // Serial.print();


  wdt_reset();  // VERY IMPORTANT this prevents the watchdog from tripping.
                // If the watchdog does trip the arduino will reset. 
                // There is 2 seconds until wdt_reset(); needs to be run or the watchdog will trip.

  lastPid = motor.controlpid(DESIRED_TARGET_DESTINATION);
  // Serial.println("Value: ", lastPid);
  Serial.println("Waiting for zero Position before reseting arduino");

  // motor.move_motor(-150);
  motor.move_motor(lastPid);
  Serial.println("Waiting for zero Position before reseting arduino");
}
