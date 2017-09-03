#include <iostream>
#include "motor_controller.h"

MotorController::MotorController() {
	std::cout << "Initializing the motor controller";
}

void MotorController::HomeMotor() {
	std::cout << "Yo we are homing";
}