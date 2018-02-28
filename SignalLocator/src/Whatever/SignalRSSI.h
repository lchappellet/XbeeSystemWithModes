#ifndef SIGNALRSSI_H_
#define SIGNALRSSI_H_

#include "Arduino.h"
#include <vector>
#include

class SignalXBee {
public:
  //RSSI SCAN CODE VARIABLES
	    //Rssi Command
	    uint8_t d0Cmd[]={'D','B'};
	    uint8_t* retVal; // variables for Xbee Rssi command
	    uint16_t retSize;
	    int rssi = 0;

	    XBee xbee = XBee();//******* need to include all the Xbee Code
	    XBeeResponse response = XBeeResponse();
	    XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x403b0afd);//Address of the Router Xbee;
	    RemoteAtCommandResponse racr = RemoteAtCommandResponse();
	    RemoteAtCommandRequest racrq = RemoteAtCommandRequest(addr64, d0Cmd);


      std::vector<double>storeRSSI(10) = 0; //creates vector a that has 10 elements for rssiSignalStregth
      std::vector<double>storeEncoder(10)= 0; //creates vector b for the correlating encoderValueNow

      // call upon encoder function to take an encoder reading as we read the rssi values
      Encoder_XBee_system encoderxbeesystem;


public:
  // Read RSSI Signal
  int readSignalStrenth();
  // send Signal
  void sendSignal(); //send a signal using the XBEE

  void readSignal(); // read a return signal using the XBEE

  void signalStoreData(int rssiSignalStrength, double encoderValueNow);

  void signalRetrieveData();

  void EMAsignalData();

  void storedEMAData(); //try to use this somehow.

  double strongestSignalEncoderDirection();
  //Things I Need
      //Store data
      //Return Average of data
      //Store Maximum value rssi


};

#endif /*SIGNALRSSI_H_*/
