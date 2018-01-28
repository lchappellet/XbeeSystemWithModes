#include "AntennaControl.h"

//constructor
Antenna_Control::Antenna_Control(){
}
//destructor
Antenna_Control::~Antenna_Control(){

}

void Antenna_Control::moveTo360Degree(){

  //This takes the motorcontrol object, and calls the controlpid function, inputs a targetDestination of zero.
  //Thus the pid motor control value is created and then it is set to move by calling upont the object motorcontrol and running the function move.

// create a while loop to check and see when the antenna has stopped moving.
// set the while loop to be true that it will only stop when it has a controlpid less than what can move the motor.
// or the motor has reached sufficiently close to its desired position

//create a variable motorControlPIDValue so I can reause the value and not call on the function multiple times.
  double motorControlPIDValue = 0;

  while(motorcontrol.controlpid(zero)>10 || encoder.read_encoder()> 5) { //could have error because antenna doesn't stop. Check this.
  //good for debugging to see the data being printed out on serial import

/*  arduino.Serial.print("DebugAntennamoveTo360")
  arduino.Serial.print(motorControlPIDValue);
  arduino.Serial.print(encoder.read_encoder());
*/
  motorControlPIDValue = motorcontrol.controlpid(zero);
  motorcontrol.move_motor(motorControlPIDValue);

  }


}
