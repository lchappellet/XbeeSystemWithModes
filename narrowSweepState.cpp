#include "narrowSweepState.h"


narrowSweepState::narrowSweepState(){

};// constructor


void narrowSweepState::narrowSweep(SignalXBee &signalxbee){ //object for xbee signal created in main function.
    antenna.moveToZeroDegrees(signalxbee);
    antenna.moveTo360Degrees(signalxbee); // during this time the antenna is also reading in data
    antenna.moveToZeroDegrees(signalxbee);
    antenna.moveTo360Degrees(signalxbee);

}

void narrowSweepState::narrowSweepRun(){ // runs all funtions of Big sweep and the xbeeStateMachine
  narrowSweep();
//  xbeeStateMachine::clearData();
//  xbeeStateMachine::didyouFindXbee();
//  xbeeStateMachine::whatState();  // output an integer which lets main function know what object to call and functions to run.

}


};
