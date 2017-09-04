#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

#include "hal.h"

namespace XbeeSystem {

  class ArduinoHal : public IHal {
    public:
      ArduinoHal();
      void init(int dig_pin, int analog_pin);
      void serial();
      void digitalWrites(int pin, bool high_voltage);
      void analogWrites(int pin, double value);
      unsigned long millis();
  };

}

#endif /* ARDUINO_HAL_H_ */
