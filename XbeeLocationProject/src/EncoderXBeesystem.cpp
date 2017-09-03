/*
 * EncoderXBeesystem.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: London
 */

#include "EncoderXBeesystem.h"

Encoder_XBee_system::Encoder_XBee_system() {
	// TODO Auto-generated constructor stub

}

double read_encoder::Encoder_XBee_system(){

	//Read Encoder and calculate time
	//gain* encoder where gain = 360/ 1 revolution of encoder counts
	//cartCurrent[0] = (.147*myEnc.read()); // gives value in degrees of where system is located for 437rpm motor
	// cartCurrent[0] = (0.002667*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using OD(outerdiameters)
	cartCurrent[0] = (0.002476 * myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using PD(Pitch diameters)//Best***
	currentPos = cartCurrent[0];
	//Serial.println("Encoders");
	//Serial.println(currentPos);
	//Serial.println(myEnc.read());

	return currentPos;


	}

Encoder_XBee_system::~Encoder_XBee_system() {
	// TODO Auto-generated destructor stub
}

