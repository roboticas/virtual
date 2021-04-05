// File:          linebot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define MAX_SPEED 6.28
#define TRESHOLD 4

int main(int argc, char** argv) {
	// create the Robot instance.
	Robot* robot = new Robot();

	// get the time step of the current world.
	int timeStep = (int)robot->getBasicTimeStep();

	// Get the motors for this robot
	Motor* leftMotor = robot->getMotor("left wheel motor");
	Motor* rightMotor = robot->getMotor("right wheel motor");

	// Activate the left and right motors
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(MAX_SPEED);
	rightMotor->setVelocity(MAX_SPEED);

	// Initialise ground facing line detection sensors
	DistanceSensor* gs0 = robot->getDistanceSensor("gs0"); // at left
	gs0->enable(timeStep);
	DistanceSensor* gs1 = robot->getDistanceSensor("gs1"); // in middle
	gs1->enable(timeStep);
	DistanceSensor* gs2 = robot->getDistanceSensor("gs2"); // at right
	gs2->enable(timeStep);

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (robot->step(timeStep) != -1) {
		// Read the sensors:
		double gs0Val = gs0->getValue();
		double gs1Val = gs1->getValue();
		double gs2Val = gs2->getValue();

		printf("gs0:%f gs1:%f gs2:%f\n", gs0Val, gs1Val, gs2Val);

		if (gs2Val < TRESHOLD) {
			// left ground sensor found white so steer right more
			rightMotor->setVelocity(MAX_SPEED);
			leftMotor->setVelocity(MAX_SPEED * 0.3);
		}
		else {
			if (gs0Val < TRESHOLD) {
				// right ground sensor found white so steer left more
				rightMotor->setVelocity(MAX_SPEED * 0.3);
				leftMotor->setVelocity(MAX_SPEED);
			}
			else {
				// Straight ahead
				rightMotor->setVelocity(MAX_SPEED);
				leftMotor->setVelocity(MAX_SPEED);
			}
		}
	};

	// Enter here exit cleanup code.
	delete robot;
	return 0;
}
