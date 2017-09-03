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
	  //Encoder Object
	Encoder myEnc(18, 19);

	double cartCurrent[0] = 0;
	double currentPos = 0;
	double read_encoder();
	Encoder_XBee_system();
	virtual ~Encoder_XBee_system();
};

#endif /* ENCODERXBEESYSTEM_H_ */
