/*
 * EncoderXBeesystem.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: London
 */
#include <Encoder.h>

#include "EncoderXBeesystem.h"

Encoder_XBee_system::Encoder_XBee_system(Encoder* encoder) {
  myEnc = encoder;
}

double Encoder_XBee_system::read_encoder(){
	//Read Encoder and calculate time
	//gain* encoder where gain = 360/ 1 revolution of encoder counts
	//cartCurrent[0] = (.147*myEnc.read()); // gives value in degrees of where system is located for 437rpm motor
	// cartCurrent[0] = (0.002667*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using OD(outerdiameters)

	cartCurrent = (0.002476 * myEnc->read()); // gives value in degrees of where system is located for 12rpm motor using PD(Pitch diameters)//Best***

	//Serial.println("Encoders");
	//Serial.println(currentPos);
	//Serial.println(myEnc.read());

	return cartCurrent;
	}

Encoder_XBee_system::~Encoder_XBee_system() {
	// TODO Auto-generated destructor stub
}

