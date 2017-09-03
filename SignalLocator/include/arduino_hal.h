#ifndef ARDUINO_HAL_H_
#define ARDUINO_HAL_H_

#import "arduino.h"
#import "arduino_hal.h"
#import "hal.h"

namespace XbeeSystem {

  class ArduinoHal : IHal {
    public:
      ArduinoHal();
      void serial() final;
      void digitalWrite(bool high_voltage) final;
      void analogWrite(double value) final;

    private:
      int dig_output_pin_;
      int analog_output_pin_;
  }

}

#endif /* ARDUINO_HAL_H_ */
