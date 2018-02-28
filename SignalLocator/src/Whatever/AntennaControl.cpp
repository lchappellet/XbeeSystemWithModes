#include "AntennaControl.h"


//constructor
Antenna_Control::Antenna_Control(){
}
//destructor
Antenna_Control::~Antenna_Control(){

}


//*******ADD THIS!!!
//********* We will also need the readings from the compass. This will be used to get the accurate angle of where the antenna is really pointing.
//********* Example: if the encoder reads 15degrees but the boat is faceing east then we know this is really equivalent to south east. by 15 degrees.... depending on how degrees are measured.



// The goal of this function is to get the antenna to move around the highest RSSI value but in a more focused span
void Antenna_Control::moveDesignatedDegreesRightAroundLocation(SignalXBee &signalxbee, double highestRSSIDirection , int degrees2){
  double motorControlPIDValue = 11;  // needs to be slightly bigger than 10 for the first while loop

  //******** IT will take a lot more time to keep recalculating the controlpid I will add a variables
          // Too take the value and put it as the comparison value for the while loop..... pidvalue = variable
  while(motorControlPIDValue>(10) || (motorControlPIDValue+2)>encoder.read_encoder()> (motorControlPIDValue-2)) { //could have error because antenna doesn't stop. Check this.

    //good for debugging to see the data being printed out on serial import
  /*  arduino.Serial.print("DebugAntennamoveTo360")
    arduino.Serial.print(motorControlPIDValue);
    arduino.Serial.print(encoder.read_encoder());
  */
  // The motorControlPIDValue will also help to continue keeping the while loop running until it is < 10
    motorControlPIDValue = motorcontrol.controlpid(highestRSSIDirection + degrees1); //+10 is so we go 10 degrees past the highest value to make sure we are correct in our scan
    motorcontrol.move_motor(motorControlPIDValue);
    //stores xbee signal data that has just been read.

    signalxbee.readSignalStrenth();


    }

}

void Antenna_Control::moveDesignatedDegreesLeftAroundLocation(SignalXBee &signalxbee, double highestRSSIDirection , int degrees2){
  double motorControlPIDValue = 11;  // needs to be slightly bigger than 10 for the first while loop

  //******** IT will take a lot more time to keep recalculating the controlpid I will add a variables
          // Too take the value and put it as the comparison value for the while loop..... pidvalue = variable
while(motorControlPIDValue>(10) || (motorControlPIDValue+2) > encoder.read_encoder() > (motorControlPIDValue-2)) { //could have error because antenna doesn't stop. Check this.

  //good for debugging to see the data being printed out on serial import
/*  arduino.Serial.print("DebugAntennamoveTo360")
  arduino.Serial.print(motorControlPIDValue);
  arduino.Serial.print(encoder.read_encoder());
*/
// The motorControlPIDValue will also help to continue keeping the while loop running until it is < 10
  motorControlPIDValue = motorcontrol.controlpid(highestRSSIDirection - degrees2); //+10 is so we go -10 degrees past the highest value to make sure we are correct in our scan
  motorcontrol.move_motor(motorControlPIDValue);
  //stores xbee signal data that has just been read
  signalxbee.readSignalStrenth();


  }

}


void Antenna_Control::moveTo360Degrees(SignalXBee &signalxbee){ //brings in object for signal that we use in the main class.

  //This takes the motorcontrol object, and calls the controlpid function, inputs a targetDestination of zero.
  //Thus the pid motor control value is created and then it is set to move by calling upont the object motorcontrol and running the function move.

// create a while loop to check and see when the antenna has stopped moving.
// set the while loop to be true that it will only stop when it has a controlpid less than what can move the motor.
// or the motor has reached sufficiently close to its desired position

//create a variable motorControlPIDValue so I can reause the value and not call on the function multiple times.
  double motorControlPIDValue = 11;

  //checks to see if PID is greater than 10
  while(motorControlPIDValue>(10) || encoder.read_encoder()> 5) { //could have error because antenna doesn't stop. Check this.
  //good for debugging to see the data being printed out on serial import

/*  arduino.Serial.print("DebugAntennamoveTo360")
  arduino.Serial.print(motorControlPIDValue);
  arduino.Serial.print(encoder.read_encoder());
*/
  motorControlPIDValue = motorcontrol.controlpid(threeSixty);
  motorcontrol.move_motor(motorControlPIDValue);
  signalxbee.readSignalStrenth(); // Reads the signal and stores the data


  }

// Set move_motor to zero so that the atenna stops its motion.
// We want to stop motion because the system has already reached its goal
// and we don't need it continuing to move with small PID values.

  motorcontrol.move_motor(0);



}


  void Antenna_Control::moveToZeroDegrees(SignalXBee &signalxbee){
    //same methode as moveTo360Degrees
    double motorControlPIDValue = 0;

    while(motorcontrol.controlpid(zero)>(10+zero) || encoder.read_encoder()> 5) {
      motorControlPIDValue = motorcontrol.controlpid(zero);
      motorcontrol.move_motor(motorControlPIDValue);
      signalxbee.readSignalStrenth(); // Reads the signal and stores the data

  }

}

void Antenna_Control::honeInRight(){
// use this function to got to the right of the highest reading rssi values
//psuedocode know the rssi number and the encoder value
      //use the encoder value and go either to the value or if possible 10 to the right.
      if(highestRSSIEncoderValue < 350){
        //add 10 degrees to value and send antenna there.
        double motorControlPIDValue = 0;

        while(motorcontrol.controlpid(highestRSSIEncoderValue)>10 || (highestRSSIEncoderValue+8)>encoder.read_encoder()) {
          motorControlPIDValue = motorcontrol.controlpid(highestRSSIEncoderValue+10);
          motorcontrol.move_motor(motorControlPIDValue);
        }
      }

      elseif(highestRSSIEncoderValue >= 350){ // the only difference is to output a note saying that there could be an object out of range.
        while(motorcontrol.controlpid(highestRSSIEncoderValue)>10 || (highestRSSIEncoderValue+10)>encoder.read_encoder()> (highestRSSIEncoderValue - 5)) {
          motorControlPIDValue = motorcontrol.controlpid(highestRSSIEncoderValue+10);
          motorcontrol.move_motor(motorControlPIDValue);
        }
        // Put here that there was a highest Endoder Value between 350 degrees and 10 degrees.
        // Try turning vehicle

        // It would be great if we could tell how much a vehicle has turned. That way we can adjust how much we scan.



      }

}
      // BELOW FOR honing left of the highestRSSIEncoderValue

      void Antenna_Control::honeInLeft(){
      // use this function to got to the right of the highest reading rssi values
      //psuedocode know the rssi number and the encoder value
            //use the encoder value and go either to the value or if possible 10 to the right.
            if(highestRSSIEncoderValue > 15){
              //add 10 degrees to value and send antenna there.
              double motorControlPIDValue = 0;

              while(motorcontrol.controlpid(highestRSSIEncoderValue)>10 || (highestRSSIEncoderValue-10)<encoder.read_encoder()) {
                motorControlPIDValue = motorcontrol.controlpid(highestRSSIEncoderValue-10);
                motorcontrol.move_motor(motorControlPIDValue);
              }
            }

            elseif(highestRSSIEncoderValue >= 350){ // the only difference is to output a note saying that there could be an object out of range.
              while(motorcontrol.controlpid(highestRSSIEncoderValue)>10 || (highestRSSIEncoderValue-10)>encoder.read_encoder()> (highestRSSIEncoderValue +5)) {
                motorControlPIDValue = motorcontrol.controlpid(highestRSSIEncoderValue-10);
                motorcontrol.move_motor(motorControlPIDValue);
              }
              // Put here that there was a highest Endoder Value between 350 degrees and 10 degrees.
              // Try turning vehicle

              // It would be great if we could tell how much a vehicle has turned. That way we can adjust how much we scan.



            }

          }

        void Antenna_Control::bigSweep(SignalXBee &signalxbee, int moreSweeps){ //use moreSweeps to add sweeps to minimum default
          // This should run a sequence of big sweeps
          // It should also take in the data and store it for use.
          for(int i=0; i<1+moreSweeps;i++ ){
          moveToZeroDegrees(signalXBee);
          moveTo360Degrees(signalXBee); // during this time the antenna is also reading in data
          moveToZeroDegrees(signalXBee);
          moveTo360Degrees(signalXbee);
          }

        }

        void Antenna_Control::narrowSweep(SignalXBee &signalxbee, int moreSweeps2, int degrees2){ //use moreSweeps to add sweeps to minimum default
          //Todo: create a function that calculates the confidence we have in the direction of the xbee and then tells us how wide to make degrees2.
                  // if there is a big spread of high values for encoder angles then we make degrees2 wider.


          // This should run a sequence of narrow sweeps
          // It should also take in the data and store it for use.
          double highestRSSIDirection = signalxbee.strongestSignalEncoderDirection();

            for(int i=0; i<1+moreSweeps2;i++ ){
                antenna.move10DegreesLeftAroundLocation(signalxbee, highestRSSIDirection, degrees2);
                antenna.move10DegreesRightAroundLocation(signalxbee, highestRSSIDirection, degrees2);
                antenna.move10DegreesLeftAroundLocation(signalxbee, highestRSSIDirection, degrees2);
                antenna.move10DegreesRightAroundLocation(signalxbee, highestRSSIDirection, degrees2);
                antenna.move10DegreesLeftAroundLocation(signalxbee, highestRSSIDirection, degrees2);
                antenna.move10DegreesRightAroundLocation(signalxbee, highestRSSIDirection, degrees2);

            }

        }


        void Antenna_Control::stopAntenna(){ //Stop the antenna motor. This is useful if there is some reason we need the antenna to stop moving.
        motorcontrol.move_motor(0); // tell antenna to stop moving

        }



}
