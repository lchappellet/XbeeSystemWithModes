/*
 * ScanXbeeSignal.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#include "ScanXbeeSignal.h"

ScanXbeeSignal::ScanXbeeSignal() {
	// TODO Auto-generated constructor stub

}

ScanXbeeSignal::~ScanXbeeSignal() {
	// TODO Auto-generated destructor stub
}



void  scan(){



  //****Scan from 130-180 degrees
  //tells motor to move the correct amount
  //Gathers the data into five different locations in the array.
//check to see if the data gathered is higher than the values in the array

//think about using Gps clock to help gather data when out on the water.
//this will help with trouble shooting.


  //readRSSI(); //reads the RSSI value while scanning  //Might have trouble reading both data sets
  NowRssi = rssi; // sets NowRssi to rssi value
  NowAngle = NowAngle; // gets NowAngle from Encoder reading of angle
for (int i = 0; i < 5; i++){  //for loop to compare all past values of Rssi to the new value //***** Chris saw that the values will always start with the hightest number.
                              ///***** For loop is getting each value to the same number because when the first number is the same it goes to the next number in the array.
  if (NowRssi > ScanArrayRssi[i]){ //Makes sure that the new Rssi is bigger than a scan arrayRssi before switching locations
        ScanArrayRssi[i]  = NowRssi; //switches higher value Rssi with for the older value
        ScanAngleArray[i] = NowAngle;
        break;
      }

//      else if (NowRssi == ScanArrayRssi[i]){
//        EqualRssi = EqualRssi + 1;
//
//      }

}

//debugging print out ScanArrayRssi
            Serial.println("Rssi then EncoderAngle");
            Serial.println(NowRssi);
            Serial.println(NowAngle);
          Serial.println("ScanArray");
          for (int i = 0; i < 5; i++){
                Serial.println(ScanArrayRssi[i]);

             }
//             //debugging print out ScanArrayRssi
            Serial.println("ScanAngleArray");
            Serial.println(ScanAngleArray[0]);
          for (int i = 0; i < 5; i++){
                Serial.println(ScanAngleArray[i]);

             }

//back to code
 // maxArrayValue  = getIndexOfMaximumValue(ScanArrayRssi,5); //the function to get highest value isnt working.
            maxArrayValue  = ScanArrayRssi[0];
            maxArrayValueAngle = ScanAngleArray[0];
//Debugging maxArrayValue

            Serial.println("maxArrayValue");
            Serial.println(maxArrayValue);

//back to code
  //if anything has gotten higher values then the array will change.
  //might want to use this array to see if it is the same and we are getting only low readings

//  for (int i = 0; i <LenArray ; i++){//Get the max Rssi value and the angle for that value
//        if (maxArrayValue == ScanArrayRssi[i]){
//            maxArrayValueAngle = ScanAngleArray[i];
//                                                  //Serial.print("Found Max array Value")
//           }
//     }

                  //maxArrayValue = maximum(ScanArrayRssi,5); //get the maximum Rssi value

 // Debugging
//            Serial.println("maxArrayValue"); //Find out what are the largest Rssi and Angle values
//            Serial.println(maxArrayValue);
//            Serial.println("maxArrayValueAngle");
//            Serial.println(maxArrayValueAngle);


//Back to code
// Now we have the highest RSSI value and the corresponding angle
//*** Send this data to compass and have an indicator on the LCD screen
//*** compare this data with Compass to help guide vehicle
  if (maxArrayValue < 20 || maxArrayValue > 90 ){// values are percentages out of 100 and will show that there is too much or too little signal to use effectively

          maxArrayValue = 0; // This will make shortScan not work if it is accessed
     }

}



void shortScan(){

  if (maxArrayValue < 20 || maxArrayValue > 90 ){// values are percentages out of 100 and will show that there is too much or too little signal to use effectively
    Serial.println("RESET IF STATEMENT INITIATED");
    Serial.println("maxArrayValue");
    Serial.println(maxArrayValue);

      setValue = 0; // this resets to the loop to the scan sequence
                // if maxArrayValue is == 0 then the value is not arround and we need to do the Scan function
      maxArrayValue = 0;
}
else if(20<maxArrayValue<90){
    // sets the location of the motor around the correct angle


  // run else, if max array value is in between the values above.

 //***move motor 30 degrees on either side of the MaxArrayValue

  //***scan Rssi Values as antennea moves

     //going to start again with an empty arrays.

  //need to account for when we dont see any data.


// setting up variables with values to be used to fill arrays.
shortScanNowRssi = rssi;
shortScanNowAngle = NowAngle;

//think about using Gps clock to help gather data when out on the water.
//this will help with trouble shooting.
 for (int i = 0;i < LenArray; i++){//for loop to compare all past values of Rssi to the new value

   if (shortScanNowRssi > shortScanArrayRssi[i]){ //Makes sure that the new Rssi is bigger than a scan arrayRssi before switching locations
       shortScanArrayRssi[i] = shortScanNowRssi; //switches higher value Rssi with for the older value
       shortScanAngleArray[i] = shortScanNowAngle;

       //Debugging
       Serial.println("Entered Shortscan if statement");

       break;
      }
//   else if (shortScanNowRssi == shortScanArrayRssi[i]){
//          shortScanEqualRssi = shortScanEqualRssi + 1;
//
//      }

  }

 //Debugging
   Serial.println("shortScanArrayRssi");
          for (int i = 0; i < 5; i++){
                Serial.println(shortScanArrayRssi[i]);

             }
//             //debugging print out ScanArrayRssi
                Serial.println("shortScanAngleArray");

          for (int i = 0; i < 5; i++){
                Serial.println(shortScanAngleArray[i]);

             }

   if (setValue > 5){
     maxArrayValue  = shortScanArrayRssi[0];//getIndexOfMaximumValue(shortScanArrayRssi,5);
   }

  //if anything has gotten higher values then the array will change.
  //might want to use this array to see if it is the same and we are getting only low readings
 for (int i = 0 ;i < LenArray; i++){//Get the max Rssi value and the angle for that value
     if (maxArrayValue == shortScanArrayRssi[i]){
         maxArrayValueAngle = shortScanAngleArray[i]; //needs to be the same variable so the motor starts on the same position
                                               //Serial.print("Found Max array Value")
       }
   }
     // maxArrayValue = maximum(shortScanArrayRssi,5); //get the maximum Rssi value and keep same variable as scan funtion


     // Now we have the highest RSSI value and the corresponding angle
     // *** Send this data to compass and have an indicator on the LCD screen
     // *** compare this data with Compass to help guide vehicle
   }
}
