/*
 * GpsLocationCalc.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef GPSLOCATIONCALC_H_
#define GPSLOCATIONCALC_H_

class GpsLocationCalc {
public:
	//**** GPS CODE Variables

	SoftwareSerial mySerial(53, 52); //Arduino UnoInitialize SoftwareSerial, and tell it you will be connecting through pins 7 and 8
	Adafruit_GPS
	GPS (&mySerial); //Create GPS object

	//#define mySerial Serial1 //Necessary for Arduino Mega
	//Adafruit_GPS GPS(&mySerial); //Necessary for Arduino Mega

	// Like Parse Gps example
	// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
	// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true

	// this keeps track of whether we're using the interrupt
	// off by default!
	boolean usingInterrupt = false;
	void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
	//

	String NMEA1;  //We will use this variable to hold our first NMEA sentence
	String NMEA2;  //We will use this variable to hold our second NMEA sentence
	char cc;       //Used to read the characters spewing from the GPS module

	float timer = 0.0;
	//Store Gps Locations during calculations
	float GpsLat1 = 0.0; // gives Gps locations.
	float GpsLon1 = 0.0;
	float GpsLat2 = 0.0;
	float GpsLon2 = 0.0;
	float GpsLat3 = 0.0; // Gps 3's are for the new approx. xbee location.
	float GpsLon3 = 0.0;

	//To calculate the distance between GPS locations
	float setGPS1 = 1; //this helps us set the first GPS location and keep those values
	float distanceMeters = 0.0;
	float CurLat = 0.0;
	float CurLon = 0.0;

	//Calculate the bearing of the boat.
	float yCoord = 0.0;
	float xCoord = 0.0;
	float boatBearing = 0.0;

	//Calculate Gps loc of a point knowing distance and bearing from another Gps location

	float r_CurLon;
	float r_CurLat;
	float Bearing = 45; // Bearing of travel //**** IS this used?
	float r_Bearing;
	float Distance = 0.0; // km per update
	int Eradius = 6371; // mean radius of the earth

	//Variables from the larger code
	//float maxArrayValue = 21;
	//float maxArrayValueAngle = 0.0;

	//approx calculations of xbee location and bearing
	float firstAngle = 0.0;
	float secondAngle = 0.0;
	float firstInnerAngle = 0.0;
	float secondInnerAngle = 0.0;
	float currentApproxBearing = 0.0;
	float approxDistanceXbee = 0.0;
	float currentApproxDistance = 0.0;

public:
	GpsLocationCalc();
	virtual ~GpsLocationCalc();
};

#endif /* GPSLOCATIONCALC_H_ */
