// MOTOR CONTROL ARDUINO

// #include "hal.h"
#include "arduino_hal.h"
#include "MotorControl.h"
#include <Encoder.h>
#include "EncoderXBeeSystem.h"

using namespace XbeeSystem;

const int DIG_IN_PIN = 11;
const int DIG_IN_2_PIN = 9;
const int INPUT_ENABLE_PIN = 12;
const int DESIRED_TARGET_DESTINATION = 150;

ArduinoHal hal;
Encoder encoder(1, 2);
Encoder_XBee_system xbee_encoder(&encoder);

MotorControl motor(&hal, &xbee_encoder, DIG_IN_PIN, DIG_IN_2_PIN, INPUT_ENABLE_PIN);

double lastEncoder = 0;
double lastPid = 0;

void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:
  // lastEncoder = xbee_encoder.read_encoder();
  // Serial.print();

  lastPid = motor.controlpid(DESIRED_TARGET_DESTINATION);

  motor.move_motor(lastPid);
}
