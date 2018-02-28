// MOTOR CONTROL ARDUINO
#include "arduino_hal.h"
#include "MotorControl.h"
#include "AntennaControl.h"
#include "bigSweepState.h"
// creating objects
MotorControl motor;
Antenna_Control antenna;
SignalXbee signalxbee;
SignalXbee signalXBee20meters;
bigSweepState bigsweepstate;
//creating Variables
    // stores the current direction signal for the XBee
double signalDirectionOfXBEE;


double firstAngleDirectionOfXBEE = 0;  //This variable holds the value for the firstAngle from the antenna to the Xbee
double secondAngleDirectionOfXBEE = 0; // This is the variable that holds the second angle to the xbee once we are far enough away from the first angle.


// set the minimum RSSI signal strength that we need to detect to know there is an Xbee in the vacinity
int setMinimumRSSIsignal = 20; // value between 0 to 100 with 0 as the lowest.


// retrybigSweep(used in switch case in main function) is a variable int which keeps track how many times we have run bigSweep and not found an XbEE
// If we dont find an xbee after a certain amount of tries then we want to reset the system. get rid of data stored. start again.
// We might want to do an LED notification to let people know we are resetting.
int retrybigSweep = 3;  // currently set to 3 tries in a row before we reset the whole system.


void setup() {

          Serial.begin(9600);
          // put your setup code here, to run once:
        //  motor = new MotorControl();
        GPS.begin(9600);       //Turn GPS on at baud rate of 9600
        GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
        //useInterrupt(true);
        delay(1000);  //Pause

        Serial3.begin(9600);
        // now that they are started, hook the XBee into
        //the Serial3 pins
        xbee.setSerial(Serial3);
        Serial.println("Initialization all done!");
        }

void loop() {
  // put your main code here, to run repeatedly:
  motor.move_motor(50);
  delay(500);
  motor.move_motor(0);

  bigsweepstate.bigSweepRun(signalxbee);

  //***** Verify here that we still have a high RSSI value in our stored data.
  // ***** If true then continue If false then don't and do another large scan.

// This can go into the narrow sweep class
// This would be because it has passed the change state.
  signalDirectionOfXBEE = signalxbee.strongestSignalEncoderDirection(vector<double> * storeRSSI, vector<double> * storeEncoder);



  //***** Verify here that we still have a high RSSI value in our stored data.

  //****if true then do another short scale antenna read.

 //**** set the gps location we are at and store in data.
 //*** store value of the angle at the current GPS Location in an array.
 //**** Send a signal to user that they can begin to move because you have one value.
 // **** If they don't move then keep scanning. Use Accellerometer to read if they are moving.

  //****If false then do a large scale search.
  firstAngleDirectionOfXBEE = signalxbee.strongestSignalEncoderDirection(vector<double> * storeRSSI, vector<double> * storeEncoder);

//**** Idea: use a case statement to go through state machine of when to do big sweep or narrow sweeps.

//This needs to be inside an if statement so it is only activated once.
antennabigSweep180Degrees = antenna.bigSweep180DegreesSearchforXbee();


//This if statement checks to see if anything was found during the sweep
if(didYouFindXBee == true && antennabigSweep180Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.narrow10DegreeSweepSearchforXbee(); //This function will do
  antennabigSweep180Degrees = false; // this way the if statment only activates when the antenna has done a big sweep
}

//This needs to be inside an if statement so it is only activated once.
antennaNarrowSweep10Degrees = antenna.narrow10DegreeSweepSearchforXbee();//This needs to be inside an if statement so it is only activated once.

//This statement checks that the narrow10DegreeSweepSearchforXbee has seen an xbee. Thus we can continue searching in that area.
if(didYouFindXBee == true && antennaNarrowSweep10Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.narrow10DegreeSweepSearchforXbee(); //This function will do
  antennaNarrowSweep10Degrees = false; // this way the if statment only activates when the antenna has done a big sweep
}

//This statement checks that the narrow10DegreeSweepSearchforXbee has seen an xbee. Thus we can continue searching in that area.
if(didYouFindXBee == false && antennaNarrowSweep10Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.bigSweep180DegreesSearchforXbee(); //This function will now do a big sweep with the antenna because we are not sure of the
  antennabigSweep180Degrees = false; // this way the if statment only activates when the antenna has done a big sweep
}

if (distance == 20meters && didYouFindXBee == false && antennaNarrowSweep10Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.bigSweep180DegreesSearchforXbee(); //This function will now do a big sweep with the antenna because we are not sure of the
  antennabigSweep180Degrees = false; // this way the if statment only activates when the antenna has done a big sweep

}

//This if statement checks to see if anything was found during the sweep
if(distance == 20meters && didYouFindXBee == true && antennabigSweep180Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.narrow10DegreeSweepSearchforXbee(); //This function will do
  antennabigSweep180Degrees = false; // this way the if statment only activates when the antenna has done a big sweep
}

//This statement checks that the narrow10DegreeSweepSearchforXbee has seen an xbee. Thus we can continue searching in that area.
if(distance == 20meters && didYouFindXBee == true && antennaNarrowSweep10Degrees=true){ //antenna.bigSweep180DegreesSearchforXbee() should return true false. That way I know if it recently ran. I need someway to reset them maybe the variable?
  antenna.narrow10DegreeSweepSearchforXbee(); //This function will do
  antennaNarrowSweep10Degrees = false; // this way the if statment only activates when the antenna has done a big sweep
}

//  this shal be when we start calculating the sweep for the secondAngleDirectionOfXBEE

}

enum xbeeState {bigSweep = 1,narrowSweep,mediumSweep,stopSweep,resetSweep,meters20BigSweep,meters20NarrowSweep,meters20MediumSweep, distancetoXbeeSweep, continueDistancetoXbeeSweep} state; //starts bigSweep variable at = 1 so the next is =2.....

while(sweepTrue){
//TOdo:  During each loop we need to check for the stop button being pressed or the restart button being pressed.
          // It would be best if we can have this be checked all the time because I wouldn't want to have to wait for the antenna to finish a big sweep before stopping.
          // Idea: have the stop button be wired to something that overides everything.

//Todo: we might want to run a function that calculates the distance traveled and if it is 20meters from the our agreed best angle then we will switch to the second data set.


if(retrybigSweep > 3){
  state = resetSweep;
  //Todo: also give warning why this is happening to the user.

}





      switch(state){
        case bigSweep:
        //functions go here.
        antenna.bigSweep(signalXBee, 0); //this will run the bigSweep function and update the signalXBee object data vectors
                                        // the zero is for how many multiples of sweeps we want to run.
        //Todo: I think we need to run GPS at the same time as the function above so we know the gps coordinates when we took this data.


        //if statment with function to check and see if we find an xbee signal.
        if (signalXBee.didYouFindXBee( setMinimumRSSIsignal )) { // didyouFindXbee will give true or false as a return.
          state = 2; // if true we want to run a narrowSweep to gather more accurate data.
          FirstangleSweepTrue = FirstangleSweepTrue + 1;

        }
        else {
          state = 1; // this is redundant because state is already 1. but we do want to run bigSweep again.
          retrybigSweep = retrybigSweep + 1; // This increments big sweep so we know how many times in a row it has run.

        }

        //Goal of this if statement is to check to see if data is ready for the first angle to be calculated.
        if(FirstangleSweepTrue >= 2 ){ // make 2 a variable that can be increased or decreased. That way this can be adjusted by users.
          //Todo: Call upon function that sets the firstAngle data into secured location to be used in calculations.
                // Including GPS data.
          state = meters20NarrowSweep;
          //TOdo: Set off indicator lights tells people we are ready to move and display angle of Xbee position.

        }


        break;

        case narrowSweep: // This case will run narrowSweep so that we can focus in the direction of the highest RSSI signal
                          // Hopefully if we get even better data it will make our direction calculations that much better.

        antenna.narrowSweep(signalXBee,0, 10); // This will run narrow sweep and update the data vectors
        //Todo: I think we need to run GPS at the same time as the function above so we know the gps coordinates when we took this data.


        //Issue: I think we have to use other vectors for holding the narrow sweep data. Then if we see the signal we should transfer that data into our original data.
        // This would include transfering our GPS coordinates when we took the data.


        //if statment with function to check and see if we find an xbee signal.

        if (signalXBee.didYouFindXBee( setMinimumRSSIsignal )) { // didyouFindXbee will give true or false as a return.
            FirstangleSweepTrue = FirstangleSweepTrue + 1;
          state = 2; // if true we want to keep running a narrowSweep to gather more accurate data.
        }

        else {
          state = 1; // we do want to run bigSweep. because we haven't seen the signal
          retrybigSweep = 0; // set how many times we have run bigSweep in a row to zero. This makes us not reset the whole system.

        }


        //Goal of this if statement is to check to see if data is ready for the first angle to be calculated.
        if(FirstangleSweepTrue >= 2 ){ // make 2 a variable that can be increased or decreased. That way this can be adjusted by users.
          //Todo: Call upon function that sets the firstAngle data into secured location to be used in calculations.
                // Including GPS data.
          state = meters20NarrowSweep;
          //TOdo: Set off indicator lights tells people we are ready to move and display angle of Xbee position.

        }

        break;


        //might not need the medium sweep because it is really the same as narrow sweep with a widder angle.
        case mediumSweep:
        //functions go here.
        // antenna.mediumSweep // This will run mediumSweep to see if we can find the Xbee signal
        // if we find a signal then run Narrow sweep and check to see if we have reached 20 meters, if not run bigSweep.

        // if we dont find signal check to see if we have reached 20meters if we have then run meters20BigSweep
        break;


        case stopSweep:
        //set the previous state so we can resume from the state before we stopSweep.
        previousState = currentstate; // state will get set because our system is always checking to see if the stop button has been pressed. when it has it will set priviousState to the currentstate

        //function tells antenna to stop
        anntenna.stopAntenna();
        if (resumeButtonPressed == TRUE){  // if this button is pressed then we know the person is ready to resume.
          state = previousState // set state to previous state and we can start where we left off.
          //ISSUE: we might want to start at a big sweep state if we have moved far from our current postion.

        }

        else{
          state = stopSweep; // This will keep the system perpetually in stop state.
        }
        // This is a button that can be activated to stop the sweep motion of the cart.
        // save the previous state. That way when the resume button is pressed we can switch to that state.
        // Check to see if the system resume button has been pressed. Then we can set the state to resume sweep.
        break;

        case resetSweep:
        // TOdo: will need to store some of the data on a hard drive that way it call still be used or learned from later.
        // reset all the data and save any final direction data in the hard drive/ssd


        // Run the function to clear all siganla data. >> make sure the function correctly deletes all data.
        signalxbee.clearstoredSignalData();
        state = 1;  // set system state to bigSweep

        //in some cases we would like to resart everything which includes setting the sweepTrue Variable to False.
        break;

        case meters20BigSweep:  // same as bigSweep but with different object so we don't overlap data. **** Maybe we should use a different vector instead?

        //functions go here.
        antenna.bigSweep(signalXBee20meters, 0); //this will run the bigSweep function and update the signalXBee object data vectors
                                        // the zero is for how many multiples of sweeps we want to run.
        //Todo: I think we need to run GPS at the same time as the function above so we know the gps coordinates when we took this data.


        //if statment with function to check and see if we find an xbee signal.
        if (signalXBee.didYouFindXBee( setMinimumRSSIsignal )) { // didyouFindXbee will give true or false as a return.
          secondAngleSweepTrue = secondAngleSweepTrue + 1;
          state = meters20NarrowSweep; // if true we want to run a narrowSweep to gather more accurate data.
          //TOdo: calculate GPS of current location.
          //Todo: begin calculating distance to the Xbee
        }

        else {
          state = meters20BigSweep; // this is redundant because state is already 1. but we do want to run bigSweep again.
          retrybigSweep = retrybigSweep + 1; // This increments big sweep so we know how many times in a row it has run.

        }

        //Goal of this if statement is to check to see if data is ready for the first angle to be calculated.
        if(secondAngleSweepTrue >= 2 ){ // make 2 a variable that can be increased or decreased. That way this can be adjusted by users.
          //Todo: Call upon function that sets the firstAngle data into secured location to be used in calculations.
                // Including GPS data.
          state = displayDistancetoXbee; //once we have both data points we are ready to calculate and display the XBEE distance.
          //TOdo: Set off indicator lights tells people we are ready to move and display angle of Xbee position.

        }

         // function to check and see if we find an xbee signal.
        //if we do see a signal we should begin to use the signal to narrow sweep in t  he area.
        // Also if we see a signal then we should begin to calculate the distance to the xbee
         // print out distance to xbee on screen.

        //increase how many times you have run this meters20BigSweep.
                // iF you have run big sweep 4 times in a row then reset the system.
                  // If we do reset the system we should notify the user.
        //
        break;

        case meters20NarrowSweep:
        antenna.narrowSweep(signalXBee20meters,0, 10); // This will run narrow sweep and update the data vectors
        //Todo: I think we need to run GPS at the same time as the function above so we know the gps coordinates when we took this data.


        //Issue: I think we have to use other vectors for holding the narrow sweep data. Then if we see the signal we should transfer that data into our original data.
        // This would include transfering our GPS coordinates when we took the data.


        //if statment with function to check and see if we find an xbee signal.

        if (signalXBee.didYouFindXBee( setMinimumRSSIsignal )) { // didyouFindXbee will give true or false as a return.
          secondAngleSweepTrue = secondAngleSweepTrue + 1;
          state = meters20NarrowSweep; // if true we want to keep running a narrowSweep to gather more accurate data.
          //TOdo: calculate GPS of current location.
          //Todo: begin calculating distance to the Xbee

        }

        else {
          state = meters20BigSweep; // we do want to run bigSweep. because we haven't seen the signal
          retrybigSweep = 0; // set how many times we have run bigSweep in a row to zero. This makes us not reset the whole system.

        }

        //Goal of this if statement is to check to see if data is ready for the first angle to be calculated.
        if(secondAngleSweepTrue >= 2 ){ // make 2 a variable that can be increased or decreased. That way this can be adjusted by users.
          //Todo: Call upon function that sets the firstAngle data into secured location to be used in calculations.
                // Including GPS data.
          state = displayDistancetoXbee; //once we have both data points we are ready to calculate and display the XBEE distance.
          //TOdo: Set off indicator lights tells people we are ready to move and display angle of Xbee position.

        }


        // run antenna.narrowSweep(signalXBee20meters, 0);
        // check to see if we see the SignalLocator
        // calculate the distance to the XbEE.
        // post all three values, Average location, 1st location, 2nd location xbee.
            // If you dont find the xbee then go back to the meters20BigSweep

        //
        break;

        case meters20MediumSweep:
        // run antenna.mediumSweep(signalXBee20meters, 0);
        // check to see if we see the SignalLocator
        // calculate the distance to the XbEE.
        // post all three values, Average location, 1st location, 2nd location xbee.
            // If you dont find the xbee then go back to the meters20BigSweep
        break;

        case continueDistancetoXbeeSweep:
        //once we have three values for the xbeeDistance the system will continue to sweep to point toward the xbee. However it will stop taking data.
        break;

        case displayDistancetoXbee:
        // Calculate Distance to XBee using First and Second Angle Data.
        // Display the distance to the Xbee and ask if the user wants to resume the Distance to Xbee.
        //

        break;

      }

}




}
