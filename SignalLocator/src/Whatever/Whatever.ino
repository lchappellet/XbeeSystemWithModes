// MOTOR CONTROL ARDUINO

// #include "hal.h"
#include "arduino_hal.h"
#include "MotorControl.h"

using namespace XbeeSystem;

ArduinoHal hal;
MotorControl* motor;

void setup() {
  // put your setup code here, to run once:
  hal.init(11, 9);
  motor = new MotorControl();
  motor->hal_ = &hal;
}

void loop() {
  // put your main code here, to run repeatedly:
  motor->move_motor(100);
  delay(50);
  motor->move_motor(0);
  // delay(1000);
  // motor->move_motor(-200);
  // motor->move_motor(0);
  // delay(50);
}
