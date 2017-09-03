#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

// #import "arduino.h"
#include "hal.h"

namespace XbeeSystem {

  class ArduinoHal : public IHal {
    public:
      ArduinoHal();
      ArduinoHal(int dig_output_pin, int analog_output_pin);
      void init(int dig_pin, int analog_pin);
      void serial();
      void digitalWrites(int pin, bool high_voltage);
      void analogWrites(int pin, double value);
      unsigned long millis();

    private:
      int dig_output_pin_;
      int analog_output_pin_;
  };

}

#endif /* ARDUINO_HAL_H_ */
