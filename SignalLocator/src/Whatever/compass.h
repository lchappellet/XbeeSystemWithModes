//The compass is critical in our calculations of angles.
  // If we rotate the boat(vehicle) as we are moving to our second location this will effect
  // the angle at which we see the xbee with our antenna that is fixed to the vehicle.
  // The compass gives us a frame of reference to compare each angle we collect from the antenna.
  // no matter what direction the boat is in we always will know what direction is the angle we read with the antennaControl

  #ifndef compass_H_
  #define compass_H_

#include <HMC5883L.h>

class Compass {

public:
  Compass();
  virtual ~Compass();
  HMC5883L compass; //creates compass object from HMC5883L library class
  int fixedHeadingDegrees; // Used to store Heading value
  float headingDegrees = 0.0;  // measurement of the heading


//Setup Code for the Compass and Neopixels
    //**** put this code in the constuctor?
     Serial.println("Initialize HMC5883L");
    while (!compass.begin())
      {
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(500);

      }
          // Set measurement range
      compass.setRange(HMC5883L_RANGE_1_3GA);

      // Set measurement mode
      compass.setMeasurementMode(HMC5883L_CONTINOUS);

      // Set data rate
      compass.setDataRate(HMC5883L_DATARATE_30HZ);

      // Set number of samples averaged
      compass.setSamples(HMC5883L_SAMPLES_8);

      // Set calibration offset. See HMC5883L_calibration.ino
      compass.setOffset(-5, -113);


private:





};







  #endif
