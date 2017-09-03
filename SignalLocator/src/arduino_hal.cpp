#import <Arduino.h>

#import "arduino_hal.h"
#import "hal.h"

namespace XbeeSystem {
  AruinoHal::AruinoHal(int dig_output_pin, int analog_output_pin) {
    dig_output_pin_ = dig_output_pin;
    analog_output_pin_ = analog_output_pin;
  }

  void AruinoHal::digitalWrite(bool high_voltage) {
    Arduino::digitalWrite(dig_output_pin_, high_voltage);
  }

  void AruinoHal::serial() {
    //todo fill in
  }

  void AruinoHal::analogWrite(double value) {
    analogWrite(pin, high_voltage);
    //todo fill in
  }
}
