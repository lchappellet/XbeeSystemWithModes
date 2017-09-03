/*
 * EncoderXBeesystem.h
 *
 *  Created on: Sep 2, 2017
 *      Author: London
 */

#ifndef ENCODERXBEESYSTEM_H_
#define ENCODERXBEESYSTEM_H_

#include <Encoder.h>

class Encoder_XBee_system {
public:
	Encoder_XBee_system();
	virtual ~Encoder_XBee_system();

  double read_encoder();

  double cartCurrent = 0;
  double currentPos = 0;
  
private:
  Encoder* myEnc;// = Encoder(5, 5);
};

#endif /* ENCODERXBEESYSTEM_H_ */
