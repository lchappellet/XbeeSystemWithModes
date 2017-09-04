// #include "Arduino.h"
#include <Arduino.h>

#import "hal.h"
#import "arduino_hal.h"

using namespace XbeeSystem;

// namespace XbeeSystem {
  ArduinoHal::ArduinoHal() {
    //todo nothing
  }

  // ArduinoHal::ArduinoHal(int dig_output_pin, int analog_output_pin) {
  //   dig_output_pin_ = dig_output_pin;
  //   analog_output_pin_ = analog_output_pin;
  // }

  // void ArduinoHal::init(int dig_pin, int analog_pin) {
  //   dig_output_pin_ = dig_pin;
  //   analog_output_pin_ = analog_pin;
  // }

  void ArduinoHal::digitalWrites(int pin, bool high_voltage) {
    digitalWrite(pin, high_voltage);
  }

  void ArduinoHal::digitalReads(int readpin){
    digitalRead(readpin);
  }

  void ArduinoHal::Serials() {
    //todo fill in
  }

  void ArduinoHal::analogWrites(int pin, double value) {
    analogWrite(pin, value);
  }

  unsigned long ArduinoHal::timemillis() {
    unsigned long mils = millis();
    // Serial.println("Mils");
    // Serial.print(mils);
    return mils;
  }
// }
