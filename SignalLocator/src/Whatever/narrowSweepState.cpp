#include "narrowSweepState.h"


narrowSweepState::narrowSweepState(){

};// constructor


void narrowSweepState::narrowSweep(SignalXBee &signalxbee){ //object for xbee signal created in main function.
  // need a new object to store the data
    // Maybe create this new object by calling on a class called patern which holds the whole pattern here.

    //Goes through three passes of the signalDirectionofXBEE
    antenna.move10DegreesRightAroundLocation(signalDirectionOfXBEE);
    antenna.move10DegreesLeftAroundLocation(signalDirectionOfXBEE);
    antenna.move10DegreesRightAroundLocation(signalDirectionOfXBEE);
    antenna.move10DegreesLeftAroundLocation(signalDirectionOfXBEE);
    antenna.move10DegreesRightAroundLocation(signalDirectionOfXBEE);
    antenna.move10DegreesLeftAroundLocation(signalDirectionOfXBEE);

}

void narrowSweepState::narrowSweepRun(){ // runs all funtions of Big sweep and the xbeeStateMachine
  narrowSweep();
//  xbeeStateMachine::clearData();
//  xbeeStateMachine::didyouFindXbee();
//  xbeeStateMachine::whatState();  // output an integer which lets main function know what object to call and functions to run.

}


};
