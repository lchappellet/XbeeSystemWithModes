#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

// #import "arduino.h"
#import "hal.h"

namespace XbeeSystem {

  class ArduinoHal : IHal {
    public:
      ArduinoHal();
      void serial();
      void digitalWrite(bool high_voltage);
      void analogWrite(double value);

    private:
      int dig_output_pin_;
      int analog_output_pin_;
  };

}

#endif /* ARDUINO_HAL_H_ */