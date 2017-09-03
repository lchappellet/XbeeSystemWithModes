#import <Arduino.h>

#import "arduino_hal.h"
#import "hal.h"

namespace XbeeSystem {
  ArduinoHal::AruinoHal() { }
  // ArduinoHal::AruinoHal(int dig_output_pin, int analog_output_pin) {
  //   dig_output_pin_ = dig_output_pin;
  //   analog_output_pin_ = analog_output_pin;
  // }

  void ArduinoHal::init(int dig_pin, int analog_pin) {
    dig_output_pin_ = dig_pin;
    analog_output_pin_ = analog_pin;
  }

  void AruinoHal::digitalWrite(bool high_voltage) {
    Arduino::digitalWrite(dig_output_pin_, high_voltage);
  }

  void AruinoHal::serial() {
    //todo fill in
  }

  void AruinoHal::analogWrite(double value) {
    analogWrite(analog_output_pin_, high_voltage);
  }

  unsigned long millis() {
    return millis();
  }
}
