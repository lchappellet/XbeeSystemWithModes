/*
 * GpsLocationCalc.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#include "GpsLocationCalc.h"

GpsLocationCalc::GpsLocationCalc() {
	// TODO Auto-generated constructor stub

}

GpsLocationCalc::~GpsLocationCalc() {
	// TODO Auto-generated destructor stub
}

void GPS_Calc() {
	wdt_reset();
	//Serial.println("Mode2 running");
	// if millis() or timer wraps around, we'll just reset it
	if (timer > millis())
		timer = millis();

	//   readGps(); is a function we define below which reads two NMEA sentences from GPS
	if (millis() - timer > 2000) {  //reads GPS every 2 seconds.
		if (GPS.fix) {
			Serial.println("GPS has a fix");

			//**** Bug seams to happen when no other Xbees are found and signal strength becomes 100   *******///

			if (maxArrayValue >= 20 && setGPS1 == 1) { //Pick a value that the Rssi must be larger than before we begin to calculate the approx distance of the xbee.
			//setGPS1 helps set the first GPS location and then not reset the location.
				// maxArrayValueAngle = 45; //for testing the code. using a made up maxArray Value Angle
			// firstAngle = convertAngle(AveragedAngle); //convertAngle converts the max array angle into a bearing with respect to the compass.

				//readGPS();
				GpsLat1 = GPS.latitudeDegrees; //command from library to retrieve the latitude data
				GpsLon1 = GPS.longitudeDegrees;//command from library to retrieve the longitude data
				/* Serial.println("GpsLat1");
				 Serial.print(GpsLat1,6);
				 Serial.print(",");
				 Serial.print(GpsLon1,6);
				 Serial.println("GpsLon1");
				 setGPS1 = 10; //**FOR TESTING: this will stop this if statement from being used after the first time.
				 */
			}
			if (millis() - timer > 45000 && setGPS1 == 10) {
				firstAngle = convertAngle(AveragedAngle); //get first angle from xbee calc of angle between north and the other xbee.
				setGPS1 = 11;
			}
			//readGPS();
			//Serial.println("continually measuring");
			CurLat = GPS.latitudeDegrees;
			CurLon = GPS.longitudeDegrees;
			/*Serial.println("GpsLat1");
			Serial.print(GpsLat1, 6);
			Serial.print(",");
			Serial.print(GpsLon1, 6);
			Serial.println("GpsLon1");
			Serial.print("CurLat");
			Serial.print(CurLat, 6);
			Serial.print(",");
			Serial.print(CurLon, 6);
			Serial.println("CurLon");
			*/
			distanceMeters = calc_dist(GpsLat1, GpsLon1, CurLat, CurLon); //calculate distance between two gps locations.
			//Serial.println("distanceMeters");
			//Serial.println(distanceMeters);

			if (distanceMeters >= 15 && setGPS1 == 11) { // this resets arrays so that we can get new data before reaching 20meters.
				for (int i = 0; i < 5; i++) { // make the two arrays have zero's in them.
					shortScanArrayRssi[i] = 0;
					shortScanAngleArray[i] = 0;

				}
				setGPS1 = 12;
			}

			//if(maxArrayValue >= 20){///********need to Create something that gets data taken a new for a certain amount of time. that way data when we hit 20meters is accurate.
			if (distanceMeters >= 20) { //****need to make something that lets us know if there is no max Rssi signal and angle any more. then restart.
            //        Serial.println("distanceMeters");
            //        Serial.println(distanceMeters);
				GpsLat2 = CurLat;  //when distance is > 20 meters set the second gps location
				GpsLon2 = CurLon;  //when distance is > 20 meters set the second gps location
				//maxArrayValueAngle = 90;
				//Serial.println("firstAngle");
				//Serial.println(firstAngle);
				secondAngle = convertSecondAngle(AveragedAngle); //get second angle from xbee calc of angle between north and the other xbee.
				//Serial.println("secondAngle");
				//Serial.println(secondAngle);
				boatBearing = findMyBearing(GpsLat1, GpsLon1, GpsLat2, GpsLon2); // calculates the bearing of the boat.
				firstInnerAngle = calcFirstInnerAngle(firstAngle, boatBearing); //function that calculates the inner angle with respect to the bearing and line between two gps locations
				//Serial.println("firstInnerAngle");
				//Serial.println(firstInnerAngle);

				secondInnerAngle = calcSecondInnerAngle(secondAngle,
						boatBearing); //function that calculates the Second inner angle with respect to the bearing and line between two gps locations
				//Serial.println("secondInnerAngle");
				//Serial.println(secondInnerAngle);
				approxDistanceXbee = lawofSinesApproxDistCalc(firstInnerAngle,
						secondInnerAngle, distanceMeters); // calculates approx. distance in meters Xbee is from second gps location.
				//Serial.println("approxDistanceXbee");
				//Serial.println(approxDistanceXbee);
				Distance = approxDistanceXbee * 1 / 1000; // converts from meters to km
				//Serial.println("Distance");
				//Serial.println(Distance);
				GpslatLonofDesiredLocation(GpsLat2, GpsLon2, secondAngle,
						Distance); //**** could have issues with lat lon not being the lat lon of the boat. that should be okay. we want gps of xbee
				                   // sets GpsLat3 and GpsLon3
				//Serial.print("GpsLat3");
				//Serial.print(GpsLat3, 6);
				//Serial.print(",");
				//Serial.print(GpsLon3, 6);
				//Serial.println("GpsLon3");
				currentApproxBearing = findMyBearing(CurLat, CurLon, GpsLat3,
						GpsLon3); //approxBearingofBoatToXbee(CurLat,CurLon,GpsLat3,GpsLon3); // this function also prints value to the LCD screen.
				//Serial.println("currentApproxBearing");
				//Serial.println(currentApproxBearing);
				currentApproxDistance = calc_dist(CurLat, CurLon, GpsLat3,
						GpsLon3); //this gives the approx distance to the xbee
				//Serial.println("currentApproxDistance");
				//Serial.println(currentApproxDistance);
			}
		}

	}
}



float calc_dist(float flat1, float flon1, float flat2, float flon2){ //calculates distance between two gps locations
      float dist_calc=0;
      float dist_calc2=0;
      float diflat=0;
      float diflon=0;

      //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
      diflat=radians(flat2-flat1);
      flat1=radians(flat1);
      flat2=radians(flat2);
      diflon=radians((flon2)-(flon1));

      dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
      dist_calc2= cos(flat1);
      dist_calc2*=cos(flat2);
      dist_calc2*=sin(diflon/2.0);
      dist_calc2*=sin(diflon/2.0);
      dist_calc +=dist_calc2;

      dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

      dist_calc*=6371000.0; //Converting to meters
      //Serial.println(dist_calc);
      return dist_calc;
}

float findMyBearing(float bflat1, float bflon1, float bflat2, float bflon2){




  float r_long1 = radians(bflon1);
  float r_long2 = radians(bflon2);
  float r_lat1 = radians(bflat1);
  float r_lat2 = radians(bflat2);
    yCoord = sin(r_long2 - r_long1)*cos(r_lat2);
//  Serial.println(sin(long2 - long1),6);
    Serial.println(yCoord,6);
    xCoord = cos(r_lat1)*sin(r_lat2)- sin(r_lat1)*cos(r_lat2)* cos(r_long2-r_long1);
    Serial.println(xCoord,6);
//    Serial.println(cos(long2 - long1),6);
//    Serial.println(cos(lat2));
   float boatBearing1 = atan2(yCoord,xCoord);
   float degreesboatBearing = boatBearing1*180/3.1459;
    Serial.println("bearing in Degrees :");
    Serial.println(boatBearing1*180/3.1459);
    return degreesboatBearing;
}

void GpslatLonofDesiredLocation(float Gpslat, float Gpslon, float bearing, float Distance1 )
{
 r_CurLon = radians(Gpslon);
  r_CurLat = radians(Gpslat);
  r_Bearing = radians(bearing);
  float DestLat = asin(sin(r_CurLat)*cos(Distance1/Eradius)+cos(r_CurLat)*sin(Distance1/Eradius)*cos(r_Bearing));
  float DestLon = r_CurLon + atan2(sin(r_Bearing)*sin(Distance1/Eradius)*cos(r_CurLat),cos(Distance1/Eradius)-sin(r_CurLat)*sin(DestLat));
  DestLon = (DestLon+3*PI)/(2*PI);
  int i = DestLon;
  DestLon = (DestLon - i) * (2*PI) - PI;  // normalise to -180..+180º

  GpsLat3 = degrees(DestLat);
  GpsLon3 = degrees(DestLon);


 /*
//System.out.printf("starting at a Longitude of %f and a Latitude of %f ",CurLon,CurLat);
  Serial.print("starting at a Longitude of ");
  Serial.print(CurLon,6); //this gives up to 6 decimal places for the variable CurLon
  Serial.print(" and a Latitude of ");
  Serial.println(CurLat,6);
//System.out.printf("if we travel %f km on a bearing of %f degrees ",Distance,Bearing);
  Serial.print("if we travel ");
  Serial.print(Distance,6);
  Serial.print(" km on a bearing of ");
  Serial.print(Bearing,6);
  Serial.println(" degrees");
//System.out.printf("we end up at Longitude of %f and a Latitude of %f ",degrees(DestLon),degrees(DestLat));
  Serial.print("we end up at a Longitude of ");
  Serial.print(degrees(DestLon),6);
  Serial.print(" and a Latitude of ");
  Serial.println(degrees(DestLat),6);
  */
  //return //***look at using pointers here to get two outputs for two different variables
}


//convertAngle converts the max array angle into a bearing with respect to the compass.
float convertAngle(float Angle) {
	return Angle; //*****ratio of angles or might have to use mapping function.
	//***** might need two compasses in order to coordinate bearing of rssi direction and bearing of boat direction.
}
float convertSecondAngle(float Angle2) {
	return Angle2; //******ratio of angles or might have to use mapping function.
	//***** might need two compasses in order to coordinate bearing of rssi direction and bearing of boat direction.
}

float calcFirstInnerAngle(float angleFirst, float bearing) {
	/*Serial.println("Inside function calcFirstinnerAngle");
	Serial.println("firstangle,SecondAngle,boatbearing");
	Serial.println(angleFirst);
	Serial.println(secondAngle);
	Serial.println(bearing);
	*/
	if (firstAngle < 180 && secondAngle < 180 && bearing < 180) {
		//Serial.println("InfirstInnerAngle If Statement");
		firstInnerAngle = abs(bearing - angleFirst);

	} else if (firstAngle < 90 && secondAngle < 270 && bearing < 90) {
		firstInnerAngle = abs(bearing - angleFirst);
	} else if (firstAngle > 270 && secondAngle > 270 && bearing < 90) {
		firstInnerAngle = (bearing + (360 - angleFirst));
	} else if (firstAngle < 90 && secondAngle < 95 && bearing > 270) {
		firstInnerAngle = (angleFirst + (360 - bearing));
	} else if (firstAngle > 270 && secondAngle < 95 && bearing > 270) {
		firstInnerAngle = abs(bearing - angleFirst);
	} else if (firstAngle > 270 && secondAngle > 270 && bearing > 270) {
		firstInnerAngle = abs(bearing - angleFirst);
	} else {
		firstInnerAngle = abs(bearing - angleFirst);
	}
	return firstInnerAngle;
}

float calcSecondInnerAngle(float angleSecond, float bearing) {
	//Serial.println("firstangle,SecondAngle,boatbearing");
	//Serial.println(angleSecond);
	//Serial.println(secondAngle);
	//Serial.println(bearing);
	if (firstAngle < 95) {
		if (secondAngle < 180) {
			if (bearing < 180) {
			//	Serial.println("InSecond InnerAngle If Statement");
				secondInnerAngle = 180.0
						- 10.0 * abs(bearing - angleSecond) / 10.0;
				//Serial.println("abs(bearing - angleSecond)");
				//Serial.println(abs(bearing - angleSecond));
			}
		}
	} else if (firstAngle > 180 && secondAngle < 270 && bearing < 90) { // what happens here when you have 1xt angle 140 second angle 140 and bearing 40.
		secondInnerAngle = 180 - (bearing + (360 - angleSecond));
	} else if (firstAngle > 270 && secondAngle > 270 && bearing < 90) {
		secondInnerAngle = 180 - (bearing + (360 - angleSecond));
	} else if (firstAngle < 90 && secondAngle < 95 && bearing > 270) {
		secondInnerAngle = 180 - (angleSecond + (360 - bearing));
	} else if (firstAngle > 270 && secondAngle < 95 && bearing > 270) {
		secondInnerAngle = 180 - (angleSecond + (360 - bearing));
	} else if (firstAngle > 270 && secondAngle > 270 && bearing > 270) {
		secondInnerAngle = 180 - abs(bearing - angleSecond);
	} else {
		secondInnerAngle = 180 - abs(bearing - angleSecond);
	}
	return secondInnerAngle;
}

float lawofSinesApproxDistCalc(float firstInAngle, float secondInAngle,
		float distMeters) { // use law of sines to calculate distance from second Gps to appox. Xbee location.
	float thirdInAngle = 180 - (firstInAngle + secondInAngle);
	//Serial.println("thirdInAngle");
	//Serial.println(thirdInAngle);
	float ratioDist = distMeters / sin(radians(thirdInAngle));
	//Serial.println("ratioDist");
	//Serial.println(ratioDist);
	float distSecondGps = ratioDist * sin(radians(firstInAngle));

	//Serial.println("distSecondGps");
	//Serial.println(distSecondGps);
	return distSecondGps;
}

//Function to tell us the bearing of the approximate xbee Gps location
float approxBearingofBoatToXbee(float GpsLa1, float GpsLo1, float GpsLa2,
		float GpsLo2) {
	float approxBearing = findMyBearing(GpsLa1, GpsLo1, GpsLa2, GpsLo2); //calculates the approx. bearing
//        lcd.setCursor(0,1);
//
//        lcd.print("curApproxBearing: ");  lcd.print(approxBearing);
	return approxBearing;
}

