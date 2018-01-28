#ifndef SIGNALRSSI_H_
#define SIGNALRSSI_H_

#include "Arduino.h"
#include <vector>

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


public:
  // Read RSSI Signal
  int readSignalStrenth();
  // send Signal
  void sendSignal(); //send a signal using the XBEE

  void readSignal(); // read a return signal using the XBEE

  void signalStoreData();

  void signalRetrieveData();

  void EMAsignalData();

  void storedEMAData();
  //Things I Need
      //Store data
      //Return Average of data
      //Store Maximum value rssi


};

#endif /*SIGNALRSSI_H_*/
