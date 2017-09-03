// MOTOR CONTROL ARDUINO
#include "arduino_hal.h"
#include "MotorControl.h"
//#include <Encoder.h>

ArduinoHal hal(12, 11, 9);
MotorControl motor;


void setup() {
  // put your setup code here, to run once:
//  motor = new MotorControl();
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.move_motor(50);
  delay(500);
  motor.move_motor(0);
}
