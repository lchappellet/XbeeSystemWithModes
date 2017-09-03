#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

// #import "arduino.h"
#include "hal.h"

namespace XbeeSystem {

  class ArduinoHal : IHal {
    public:
      ArduinoHal(int dig_output_pin, int analog_output_pin);
      void serial();
      void digitalWrite(bool high_voltage);
      void analogWrite(double value);

    private:
      int dig_output_pin_;
      int analog_output_pin_;
  };

}

#endif /* ARDUINO_HAL_H_ */
