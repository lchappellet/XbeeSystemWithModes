#include "GPS.h"


GPS::GPS(){

}

GPS::readGPS(){

}

GPS::~GPS(){

}

GPS::calcFirst innerAngle(){
  // *** Need to have code for when firstAngle is equal to 180 or 0
  //      Currently I am not sure what to do in those cases. I will need to draw them up.
  //      Might need to say something about angle being 360 degrees in the code.




  // The first iff statement needs to find a way that the boat is in the same bearing direction as the XBEE
  // *** curently this equation does not work because it aways assumes the xbee is directly up or down.
  //     we need something that will recognise if the boat has moved more than +/- 90 degrees than the XBEE direction.

plus90DegreesFirstAngle  = firstAngle + 90;
minus90DegreesFirstAngle = firstAngle - 90;

if (plus90DegreesFirstAngle > 360){ // This will help adjust if the angle goes greater than 360.
    plus90DegreesFirstAngle = plus90DegreesFirstAngle - 360;
}

if (minus90DegreesFirstAngle < 0){ // This will help adjust if the angle goes less than zero.
    minus90DegreesFirstAngle = 360-minus90DegreesFirstAngle;
}

// The if statement below makes sure the bearingAgle is within +/- 90 degrees
// Currently the below statement only work when the bearing is in the same direction as the xbee by some small amount.
 if((minus90DegreesFirstAngle < bearingAngle) && (bearingAngle < plus90DegreesFirstAngle)){
      if( firstAngle >180 ){ // part of algorithm depends on this first thing being true.
           if(firstAngle < secondAngle && firstAngle>180 && secondInnerAngle >180){
             secondInnerAngle = abs(180+bearingAngle -secondInnerAngle);

           }
           else {
             secondInnerAngle = abs(180 + secondInnerAngle - bearingAngle);

           }
       }


      if( firstAngle<180 ){
          if(firstAngle > secondAngle && firstAngle<180 && secondInnerAngle <180){
            secondInnerAngle = abs (180 + secondInnerAngle - bearingAngle);

          }
          else {
            secondInnerAngle = abs (180 + bearingAngle - secondInnerAngle);

          }
       }
   }
   else{
     //***** Need to restart the search and all the variable values values.
     //      This means that the vehicle went in the opposite direction to the Xbee signal
     //   ***Eventually I need to create algritms that account for the vehicle going in the opposite direction than the Xbee.

   }
}


//Code from GPS parsing example
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  SIGNAL(TIMER0_COMPA_vect) {
     //Serial.println("Signal Activated");
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;
      //Serial.println(c);
      // writing direct to UDR0 is much much faster than Serial.print
      // but only one character can be written at a time.
  #endif
  }

  void useInterrupt(boolean v) {
            Serial.println("UseInterupt Activated");
        if (v) {
            // Timer0 is already used for millis() - we'll just interrupt somewhere
            // in the middle and call the "Compare A" function above
            OCR0A = 0xAF;
            TIMSK0 |= _BV(OCIE0A);
            usingInterrupt = true;
            }
        else {
            // do not call the interrupt function COMPA anymore
            TIMSK0 &= ~_BV(OCIE0A);
            usingInterrupt = false;
          }
      }

      //From GPs Example Parsing code
           if (! usingInterrupt) {
                // read data from the GPS in the 'main loop'
                char c = GPS.read();
                // if you want to debug, this is a good time to do it!
                if (GPSECHO){
                    if (c) {Serial.print(c);}
                 }
            }
           if (GPS.newNMEAreceived()) {
                  // a tricky thing here is if we print the NMEA sentence, or data
                  // we end up not listening and catching other sentences!
                  // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
                //  Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
                if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
                     return;  // we can fail to parse a sentence in which case we should just wait for another

                 }
             }

        useInterrupt(true);

        GPS_Calc();



        void GPS_Calc(){
          wdt_reset();
           //Serial.println("Mode2 running");
           // if millis() or timer wraps around, we'll just reset it
          if (timer > millis())  timer = millis();

           //   readGps(); is a function we define below which reads two NMEA sentences from GPS
        if (millis() - timer > 2000) {  //I believe this is to wait for the previous GPS reading to be done.
          if (GPS.fix) { //This means that we have a fix on the GPS and are ready to read data.
                   Serial.println("GPS has a fix");

             //**** Bug seams to happen when no other Xbees are found and signal strength becomes 100   *******///

              if(rssiSignalStregth >= 20 && setGPS1 == 1){//Pick a value that the Rssi must be larger than before we begin to calculate the approx distance of the xbee.
                  //setGPS1 is to prevent us from overwriting GPS values.
                   //readGPS();
                   GpsLat1 = GPS.latitudeDegrees;
                   GpsLon1 = GPS.longitudeDegrees;
                   Serial.println("GpsLat1");
                   Serial.print(GpsLat1,6);
                   Serial.print(",");
                   Serial.print(GpsLon1,6);
                    Serial.println("GpsLon1");
                   setGPS1 = 10; //**FOR TESTING: this will stop this if statement from being used after the first time.
              }
              if (millis() -timer> 45000 && setGPS1 == 10){ //not sure why this is here.?
                  firstAngle = convertAngle(AveragedAngle);
                  setGPS1 = 11;
              }
              //readGPS();
                  Serial.println("continually measuring");  //always measuring these values.
                  CurLat = GPS.latitudeDegrees; //set the current latitudeDegrees
                  CurLon = GPS.longitudeDegrees;  // set the current longitudeDegrees

// useful for Debugging
/*
                  Serial.println("GpsLat1");
                   Serial.print(GpsLat1,6);
                    Serial.print(",");
                    Serial.print(GpsLon1,6);
                    Serial.println("GpsLon1");
                  Serial.print("CurLat");
                    Serial.print(CurLat,6);
                    Serial.print(",");
                    Serial.print(CurLon,6);
                    Serial.println("CurLon");
                  distanceMeters = calc_dist(GpsLat1,GpsLon1,CurLat,CurLon);
                  Serial.println("distanceMeters");
                  Serial.println(distanceMeters);
*/

// use the below if statement so that we can reset the data before trying to scan more.
               if (distanceMeters >= 15 && setGPS1 == 11){  // this resets arrays so that we can get new data before reaching 20meters.
                   for(int i = 0; i< 5 ; i++){ // make the two arrays have zero's in them.
                        shortScanArrayRssi[i] = 0;
                        shortScanAngleArray[i] = 0;

                        }
                           setGPS1 = 12;
                    }


              //if(maxArrayValue >= 20){///********need to Create something that gets data taken a new for a certain amount of time. that way data when we hit 20meters is accurate.
              if (distanceMeters >= 20){ //****need to make something that lets us know if there is no max Rssi signal and angle any more. then restart.
                  //Debugging
          //        Serial.println("distanceMeters");
          //        Serial.println(distanceMeters);
                  GpsLat2 = CurLat;  //
                  GpsLon2 = CurLon;


                  //Debugging
                   Serial.println("firstAngle");
                   Serial.println(firstAngle);

                  // set the second angle
                  secondAngle = convertSecondAngle(AveragedAngle); //convert second angle

                  //debugging
                  Serial.println("secondAngle");
                  Serial.println(secondAngle);

                  //Calculate the boat bearing. using two gps points
                  boatBearing =  findMyBearing(GpsLat1,GpsLon1, GpsLat2,GpsLon2);// calculates the bearing of the boat.

                  // calculate firstInnerAngle of triangle
                        // This uses the side of the triangle we just traveled on and the first angle we calculated.
                  firstInnerAngle = calcFirstInnerAngle(firstAngle, boatBearing); //function that calculates the inner angle with respect to the bearing and line between two gps locations

                  Serial.println("firstInnerAngle");
                  Serial.println(firstInnerAngle);

                  secondInnerAngle = calcSecondInnerAngle(secondAngle, boatBearing);//function that calculates the Second inner angle with respect to the bearing and line between two gps locations
                  Serial.println("secondInnerAngle");
                  Serial.println(secondInnerAngle);
                  approxDistanceXbee = lawofSinesApproxDistCalc(firstInnerAngle,secondInnerAngle,distanceMeters); // calculates approx. distance in meters Xbee is from second gps location.
                  Serial.println("approxDistanceXbee");
                  Serial.println(approxDistanceXbee);
                  Distance = approxDistanceXbee*1/1000; // converts from meters to km
                   Serial.println("Distance");
                  Serial.println(Distance);
                  GpslatLonofDesiredLocation(GpsLat2, GpsLon2, secondAngle, Distance); //**** could have issues with lat lon not being the lat lon of the boat. that should be okay. we want gps of xbee
                   Serial.print("GpsLat3");
                   Serial.print(GpsLat3,6);
                   Serial.print(",");
                   Serial.print(GpsLon3,6);
                   Serial.println("GpsLon3");
                  currentApproxBearing = findMyBearing(CurLat,CurLon,GpsLat3,GpsLon3);//approxBearingofBoatToXbee(CurLat,CurLon,GpsLat3,GpsLon3); // this function also prints value to the LCD screen.
                  Serial.println("currentApproxBearing");
                  Serial.println(currentApproxBearing);
                  currentApproxDistance = calc_dist(CurLat,CurLon,GpsLat3,GpsLon3); //this gives the approx distance to the xbee
                  Serial.println("currentApproxDistance");
                  Serial.println(currentApproxDistance);
               }
              }

             }
            }
