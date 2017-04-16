/*
 * Modes.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef MODES_H_
#define MODES_H_

class Modes {
	void resetMode2(){
	  //Store Gps Locations during calculations
	 GpsLat1 = 0.0; // gives Gps locations.
	 GpsLon1 = 0.0;
	 GpsLat2 = 0.0;
	 GpsLon2 = 0.0;
	 GpsLat3 = 0.0;// Gps 3's are for the new approx. xbee location.
	 GpsLon3 = 0.0;


	//To calculate the distance between GPS locations
	 distanceMeters = 0.0;
	 CurLat = 0.0;
	 CurLon =  0.0;

	//Calculate the bearing of the boat.
	 yCoord = 0.0;
	 xCoord = 0.0;
	 boatBearing = 0.0;

	//Calculate Gps loc of a point knowing distance and bearing from another Gps location

	 r_CurLon;
	 r_CurLat;
	 Bearing = 45; // Bearing of travel
	 r_Bearing;
	 Distance = 0.0; // km per update
	 Eradius = 6371; // mean radius of the earth


	//Variables from the larger code
	 maxArrayValue = 21;
	 maxArrayValueAngle = 0.0;






	 //approx calculations of xbee location and bearing
	  firstAngle = 0.0;
	  secondAngle = 0.0;
	  firstInnerAngle = 0.0;
	  secondInnerAngle = 0.0;
	  currentApproxBearing = 0.0;
	  approxDistanceXbee = 0.0;
	  currentApproxDistance = 0.0;
	}


public:
	Modes();
	virtual ~Modes();
};

#endif /* MODES_H_ */
