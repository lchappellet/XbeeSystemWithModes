/*
 * CompassNeopixelReading.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef COMPASSNEOPIXELREADING_H_
#define COMPASSNEOPIXELREADING_H_

class CompassNeopixelReading {
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


public:
	CompassNeopixelReading();
	virtual ~CompassNeopixelReading();
};

#endif /* COMPASSNEOPIXELREADING_H_ */
