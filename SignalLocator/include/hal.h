#ifndef HAL_H_
#define HAL_H_

namespace XbeeSystem {

  class IHal {
    virtual void digitalWrite(int pin_number, bool high_voltage);
    virtual void analogWrite(int pin_number, double value);
  }

}

#endif /* HAL_H_ */
