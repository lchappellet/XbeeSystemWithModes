
#include <HMC5883L.h>
#include "compass.h"
// most likely need to #include the compass Library


//confstructor
Compass::Compass(){

}
//deconstructor
Compass::~Compass(){

}


double Compass::readCompass(){
 // Here is where I will read the compass value to see what direction the boat is pointing.

}

double Compass::storeCompassValue() {
// we might need to store the compass values here.
// Especially if we are taking averages.

}

void Compass::compassHeading(){

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

}
