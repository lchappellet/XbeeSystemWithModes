#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

#include "hal.h"

namespace XbeeSystem {

  class ArduinoHal : public IHal {
    public:
      ArduinoHal();
      void init(int dig_pin, int analog_pin);
      void Serials();
      void digitalWrites(int pin, bool high_voltage);
      bool digitalReads(int readpin);
      void analogWrites(int pin, double value);
      unsigned long timemillis();
  };

}

#endif /* ARDUINO_HAL_H_ */
