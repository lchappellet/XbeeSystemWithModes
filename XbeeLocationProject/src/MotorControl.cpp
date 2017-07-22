/*
 * MotorControl.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */
/*psudocode
 * Get to a degree then measure the heading 5 times and take the EMA each time. Then go on.
 * If Horizontal motion happens(vehicle is turning) designate when to take the measurement again.
 * Maybe when we have designated the highest rssi frequency we should then try to get an accurate heading of that direction as well.
 *
 */
#include "MotorControl.h"

MotorControl::MotorControl() {
	// TODO Auto-generated constructor stub
}

MotorControl::~MotorControl() {
	// TODO Auto-generated destructor stub
}

void CompassAntennaBearing(){
  strip.clear(); // clear all pixels
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (13.0 + (27.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
    {
      heading += 2 * PI;
    }

  if (heading > 2 * PI)
    {
      heading -= 2 * PI;
    }

  headingDegrees = heading * 180/M_PI;


 // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();



/*
 // To Fix rotation speed of HMC5883L Compass module
    if (headingDegrees >= 1 && headingDegrees < 240)
    {
    fixedHeadingDegrees = map (headingDegrees * 100, 0, 239 * 100, 0, 179 * 100) /100.00;
    }
    else {
      if (headingDegrees >= 240)
      {
      fixedHeadingDegrees = map (headingDegrees * 100, 240 * 100, 360 * 100, 180 * 100, 360 * 100) /100.00;
      }
    }
  */


  //above code was causing issues with the heading readings
 // int headvalue = fixedHeadingDegrees/18;
  // Convert to degrees
  //Serial.println(headvalue);

   int ledtoheading = map(heading*100, 0, 600, 0, 1100)/100.00;
   Serial.println(ledtoheading);



  if (ledtoheading == 0){

         strip.setPixelColor(0, strip.Color(0, 0, 255));// Blue
         strip.setPixelColor(11, strip.Color(255, 0, 0));// Red
         strip.setPixelColor(1, strip.Color(255, 0, 0));// Red

    //   leds_RING[11] = CRGB::Red;
    //   leds_RING[0] = CRGB::Green;
    //   leds_RING[10] = CRGB::Green;
      }
  else {
          if (ledtoheading == 11){
           strip.setPixelColor(11, strip.Color(0, 0, 255));// Blue
           strip.setPixelColor(10, strip.Color(255, 0, 0));// Red
           strip.setPixelColor(0, strip.Color(255, 0, 0));// Red

      //     leds_RING[0] = CRGB::Red;
      //     leds_RING[11] = CRGB::Green;
      //     leds_RING[1] = CRGB::Green;
    }
    else {
           strip.setPixelColor(ledtoheading, strip.Color(0, 0, 255));// Blue
           strip.setPixelColor(ledtoheading+1, strip.Color(255, 0, 0));// Red
           strip.setPixelColor(ledtoheading-1, strip.Color(255, 0, 0));// Red

          //      leds_RING[ledtoheading] = CRGB::Red;
          //      leds_RING[ledtoheading+1] = CRGB::Green;
          //      leds_RING[ledtoheading-1] = CRGB::Green;


          }

   }

 strip.show();
}


void CalibHeadToPosition(){
   readEncoders();
   currentPos = cartCurrent[0];   // good
 NowAngle = headingDegrees; //good

float Num = NowAngle-currentPos;
if(Num >=0 ){                               ////////// *****************not finished here!!!!! still need to complete
     int Total = Num;
     int TotalPlus180 = 180+total;

     if(TotalPlus180 > 360){
           int NewTotal = 360 - TotalPlus180;

        }
     else{
          NewTotal = TotalPlus180
         }
         if(AverageAngle>=180){
           int Degrees = map(AverageAngle, Total, 360, 0, abs(Num));
         }

           else {
         int Degrees = map(AveragedAngle,0,NewTotal,0,NewTotal);
     }



     }

  }

  if(Num < 0){
    Total = 360+Num;
    TotalPlus180 = Total+180;
      if(TotalPlus180 > 360){
        int NewTotal = 360 - TotalPlus180;
         if(headingDegrees>Total){
                int Degrees = map(headingDegrees, Total, 360, 0, abs(Num));  ////////// *****************not finished here!!!!! still need to complete
             }
         else  {
                int Degrees = map(headingDegrees,0,NewTotal,0,180);
           }
     }
      Total->TotalPlus180
     TotalPlus180 -> NewTotal
     TotalPlus180


  }
}





void readEncoders(){

//Read Encoder and calculate time
//gain* encoder where gain = 360/ 1 revolution of encoder counts
//cartCurrent[0] = (.147*myEnc.read()); // gives value in degrees of where system is located for 437rpm motor
// cartCurrent[0] = (0.002667*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using OD(outerdiameters)
cartCurrent[0] = (0.002476*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using PD(Pitch diameters)//Best***
currentPos = cartCurrent[0];
Serial.println("Encoders");
Serial.println(currentPos);
Serial.println(myEnc.read());
// NowAngle = currentPos;
// shortScanNowAngle = currentPos;
//currentPos = cartCurrent[0];
NowAngle = headingDegrees; // given by the compass. It would be better to use the compass reading to get the value of what the encoder is giving.
							// That way we can use the encoder to keep track of exactly what angle the system is pointing and the compass to keep the true heading.

shortScanNowAngle = headingDegrees;
return;

}

void controlpid(){

 //How long since we last calculated
 unsigned long timeNow = millis();
 double deltaTime = (double)(timeNow - lastTimePid);

 //Compute all the working error variables
 double error = setPointTarget - currentPos;
 errSum += (error * deltaTime);

 double dErr = (error - lastErr) / deltaTime;

 /*Compute PID Output*/
 pidControlSignal = kp * error + ki * errSum + kd * dErr;

 //Remember some variables for next time
 lastErr = error;
 lastTimePid = timeNow;

/*

*/
//   Serial.print(" Kp:");
//  Serial.print(kp*error);
//  Serial.print(" Ki:");
//  Serial.print(ki*errSum);
//  Serial.print(" Kd:");
//  Serial.println(kd*dErr);
return;
}

void moveWheels(){

if(pidControlSignal>=0){
  currentDir = LOW;
}
if(pidControlSignal<0){
  currentDir = HIGH;
}
digitalWrite(in1, !currentDir);
digitalWrite(in2, currentDir);
//  digitalWrite(in1, currentDir);       //move counterclockwise
//  digitalWrite(in2, !currentDir);

if(abs(pidControlSignal) > 120){
  pidControlSignal = 120;
}

//Serial.println(pidControlSignal);
analogWrite(enA, abs(pidControlSignal));

//  analogWrite(enB, abs(pidControlSignal));

//  unsigned long currentMillis = millis();
//
//
//  if(currentMillis - previousMillis > interval) {
//    // save the last time you blinked the LED
//    previousMillis = currentMillis;
//     analogWrite(enA, 0);
//     analogWrite(enB, 0);
//     delay(100);
//  }

return;
}


