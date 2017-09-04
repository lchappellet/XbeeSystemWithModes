#ifndef HAL_H_
#define HAL_H_

namespace XbeeSystem {

  class IHal {
    public:
      virtual void analogWrites(int pin, double value);
      virtual void digitalWrites(int pin, bool high_voltage);
      virtual void serial();

      virtual unsigned long timemillis();
  };

}

#endif /* HAL_H_ */
