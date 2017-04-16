/*
 * ReadXbeeSignalStrength.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#include "ReadXbeeSignalStrength.h"

ReadXbeeSignalStrength::ReadXbeeSignalStrength() {
	// TODO Auto-generated constructor stub

}

ReadXbeeSignalStrength::~ReadXbeeSignalStrength() {
	// TODO Auto-generated destructor stub
}



//XBEE SCAN CODE FUNCTIONS
void readRSSI(){

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
          xbee.getResponse().getRemoteAtCommandResponse(racr);
          retVal = xbee.getResponse().getFrameData();
          retSize = xbee.getResponse().getPacketLength();


      // debugging

        //      for (int i = 0; i < retSize; i++) {
        //            Serial.print(" ");
        //            Serial.print(retVal[i],HEX);
        //
        //
        //         }
          rssi = retVal[14];
          rssi = map(rssi, 0, 100, 100, 0);

          //Debugging
                        Serial.println("RSSI");
                        Serial.println(rssi);
                        //Serial.println(millis());
          }


    }



    else if (xbee.getResponse().isError()) {
           //nss.print("Error reading packet.  Error code: ");
           //nss.println(xbee.getResponse().getErrorCode());
          }

  //back to code
}


