// we will need to include the xbee library here. lots of calls to this library.
#include "SignalRSSI.h"
#include "EncoderXBeesystem.h"

//constructor
SignalXBee::SignalXBee(){

}

void SignalXBee::readSignalStrenth(){
        xbee.send(racrq);
        //delay(50);
        xbee.readPacket();
           //Serial.println("Hi You"); //Debug
        if (xbee.getResponse().isAvailable()) {
                  //Debugging
              //               Serial.println("");
              //               Serial.println("AVAILABLE");
              //               Serial.println(xbee.getResponse().getApiId());
              //               Serial.println("REMOTE_AT_COMMAND_RESPONSE");
              //               Serial.print(REMOTE_AT_COMMAND_RESPONSE);
              //               Serial.println("");

              if (xbee.getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE) {
               // xbee.getResponse().getZBRxResponse(rx);
                xbee.getResponse().getRemoteAtCommandResponse(racr); //WHAT does this do?
                retVal = xbee.getResponse().getFrameData();
                retSize = xbee.getResponse().getPacketLength();

                //***** RIGHT HERE Call on the encoder to read a value. This will have the highest
                //       chance of getting the encoder value as accurate to when the rssi value was recieved.

                double readEncoder = encoderxbeesystem.read_encoder();

              //***** .readSignalHow can I get the device ID as well in what I am reading. It is important to know what device I am talking to or searching for.
              // debugging

              //      for (int i = 0; i < retSize; i++) {
              //            Serial.print(" ");
              //            Serial.print(retVal[i],HEX);
              //
              //
              //        }

                rssi = retVal[14];
                rssi = map(rssi, 0, 100, 100, 0); // this is an arduino function that maps the rssi value between 1-100


                signalStoreData(rssi, readEncoder); //readEncoder is above and it is the encoder value at that time. Store in a vector the values.
                return 0;  //this returns the rssi value


                //Debugging
                          //    Serial.println("RSSI");
                          //    Serial.println(rssi);
                              //Serial.println(millis());
                }

          }




            else if (xbee.getResponse().isError()) {

    //NEED TO HAVE SOME PRINT FUNCTION HERE.
         //nss.print("Error reading packet.  Error code: ");
         //nss.println(xbee.getResponse().getErrorCode());
            }

   }





void SignalXBee::signalStoreData(int rssiSignalStrength, double encoderValueNow){
  // create a dictionary or a double array that is able to store each encoder value we enter.


     for (int i = 0; i<storeRSSI.size(); i++){//checks each vector value and sees if it needs to be upgraded
                if(storeRSSI(i) == 0){
                    storeRSSI(i) = rssiSignalStrength; //only if vector element is equal to moveToZeroDegrees
                    storeEncoder(i) = encoderValueNow;  // ads the corresponding encoderValue to b vector in the same Position

                    break;                   //  exits the for loop because a value has been added
                }

          else if(storeRSSI(i) < rssiSignalStrength){ // go through each element in the vector
                                              // this guarranties that the largest RSSI value is the first element.
                  storeRSSI(i) = rssiSignalStrength;// switch the element with the new highest vale
                  storeEncoder(i) = encoderValueNow; // add the corresponding encoderValueNow to the same location in the b vector
          }

      }

}



        }
 // might need to adjust the input names of the vectors below
double SignalXBee::strongestSignalEncoderDirection(vector<double> * storeRSSI, vector<double> * storeEncoder) {
      for(int i = 1; i<storeRSSI.size(); i++){  // finds what is the greatestRSSIValue
          if(storeRSSI(i) >= storeRSSI(i-1)) { // I put an equal to sign just in case they are all equal values.
            greatestRSSIValue = storeRSSI(i);
           }
          }

          std::vector<double>multipleEncoderValues(); //Stores the multiple encoder values that are the same and correspond to highest RSSI

      for(int i = 1; i<storeRSSI.size(); i++){
              if(greatestRSSIValue == storeRSSI(i)) {
                multipleEncoderValues.pushback(storeEncoder(i));// rssi value stored

               }
          }

      double sumOfEncoderValues = 0;
      double averageOfEncoderValues = 0;

      for(int i = 0; i<multipleEncoderValues.size(); i++){
            sumOfEncoderValues = sumOfEncoderValues + multipleEncoderValues(i);

      }

      averageOfEncoderValues = sumOfEncoderValues/(multipleEncoderValues.size()); // caluclates the averageOfEncoderValues

  return averageOfEncoderValues;
//take average of encoder Values
// return averageOfEncoderValues
// this will give you the angle of the highest rsssi signal.


}



//get 10 values only keep the highest 10
//analyze the values and find the highest value
//are there any multiples of the value? l-
// take the average encoder value between them all.
// That will be the value we use to get an angle.


//SCAN again honed in sweep
// when we are focused on an area we should take new set of 10 values
// compare those values to the highest value if they aren't any where close then we should start again.
// send signal to start big sweeps again.

// Then use these 10 values to pick the highest concentration of 3 Values unless we have more than 3 highest values.
// again we want to find the average angle from the encoder values.






//destructor
SignalXBee::~SignalXBee(){

}
