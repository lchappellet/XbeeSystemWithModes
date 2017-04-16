/*
 * ScanXbeeSignal.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef SCANXBEESIGNAL_H_
#define SCANXBEESIGNAL_H_

class ScanXbeeSignal {
public:
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


	ScanXbeeSignal();
	virtual ~ScanXbeeSignal();
};

#endif /* SCANXBEESIGNAL_H_ */
