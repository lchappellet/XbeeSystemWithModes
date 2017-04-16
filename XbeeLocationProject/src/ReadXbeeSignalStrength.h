/*
 * ReadXbeeSignalStrength.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef READXBEESIGNALSTRENGTH_H_
#define READXBEESIGNALSTRENGTH_H_

class ReadXbeeSignalStrength {

	/RSSI SCAN CODE VARIABLES
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


	    //Scan and ShortScan function Arrays and Variables for functions
	    int ScanArrayRssi[5];    // maybe make these global values
	    int ScanAngleArray[5];//makes an array of angles that correspond to Rssi array
	    int setValue = 0;     // These three variables keep track of how many times the system has changed directions.
	    int PreValue = 0;
	    int PreValue1 = 0;
	    int NowRssi = 0;         //make this global as well
	    int NowAngle = 0;
	    int EqualRssi = 0;
	    int  shortScanArrayRssi[5] = {0,0,0,0,0};    // maybe make these global values
	    int shortScanAngleArray[5] = {0,0,0,0,0};   //makes an array of angles that correspond to Rssi array
	    int shortScanNowRssi = 0;     //make this global as well
	    int shortScanNowAngle = 0;
	    int ShortScanEqualRssi = 0;
	    int shortScanEqualRssi =0;
	    int maxArrayValueAngle = 0;
	    int maxArrayValue = 0;
	    int LenArray =5;
	    int AveragedAngle = 0;
	    int AddedAngles = 0;


public:
	ReadXbeeSignalStrength();
	virtual ~ReadXbeeSignalStrength();
};

#endif /* READXBEESIGNALSTRENGTH_H_ */
