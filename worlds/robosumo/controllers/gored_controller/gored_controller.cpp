// File:          sumobot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <stdlib.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define MAX_SPEED 6.28

enum class States { STARTING, RUNNING, REVERSING };

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
	// Select the robot for this controller
	(void)_putenv("WEBOTS_ROBOT_NAME=sumobotred");

	// create the Robot instance.
	Robot* robot = new Robot();

	// get the time step of the current world.
	int timeStep = (int)robot->getBasicTimeStep();

	// Get the motors for this robot
	Motor* leftMotor = robot->getMotor("left wheel motor");
	Motor* rightMotor = robot->getMotor("right wheel motor");

	// Initialise ground facing, border detection sensors
	DistanceSensor* ir0 = robot->getDistanceSensor("ir0");
	ir0->enable(timeStep);
	DistanceSensor* ir1 = robot->getDistanceSensor("ir1");
	ir1->enable(timeStep);

	// Initialise proximity sensors ps0 to ps7
	DistanceSensor* ps0 = robot->getDistanceSensor("ps0");
	ps0->enable(timeStep);
	DistanceSensor* ps1 = robot->getDistanceSensor("ps1");
	ps1->enable(timeStep);
	DistanceSensor* ps2 = robot->getDistanceSensor("ps2");
	ps2->enable(timeStep);
	DistanceSensor* ps3 = robot->getDistanceSensor("ps3");
	ps3->enable(timeStep);
	DistanceSensor* ps4 = robot->getDistanceSensor("ps4");
	ps4->enable(timeStep);
	DistanceSensor* ps5 = robot->getDistanceSensor("ps5");
	ps5->enable(timeStep);
	DistanceSensor* ps6 = robot->getDistanceSensor("ps6");
	ps6->enable(timeStep);
	DistanceSensor* ps7 = robot->getDistanceSensor("ps7");
	ps7->enable(timeStep);

	// Activate the left and right motors
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(MAX_SPEED);
	rightMotor->setVelocity(MAX_SPEED);

	States state = States::STARTING;
	double startTime;
	States previousState = state;

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (robot->step(timeStep) != -1) {
		// Show change in state
		if (previousState != state) {
			printf("State: %d\n", state);
			previousState = state;
		}

		// Read the border line sensors:
		double ir0Val = ir0->getValue();
		double ir1Val = ir1->getValue();

		// Read the horizontal distance sensors
		double ps0Val = ps0->getValue();
		double ps1Val = ps0->getValue();
		double ps2Val = ps0->getValue();
		double ps3Val = ps0->getValue();
		double ps4Val = ps0->getValue();
		double ps5Val = ps0->getValue();
		double ps6Val = ps0->getValue();
		double ps7Val = ps0->getValue();


		switch (state) {
		case States::STARTING:
			// Start after one second
			if (robot->getTime() > 1)
				state = States::RUNNING;
			break;
		case States::RUNNING:
			// Test ground line sensor for border line found
			if (ir0Val > 100) {
				// Backup turn at white line border
		        printf("time:%f ir0:%f ir1:%f\n", robot->getTime(), ir0Val, ir1Val);
				leftMotor->setVelocity(-MAX_SPEED);
				rightMotor->setVelocity(-MAX_SPEED/1.1);
				state = States::REVERSING;
				startTime = robot->getTime();
			}
			break;
		case States::REVERSING:
			if ((robot->getTime() - startTime) > 3) {
				leftMotor->setVelocity(MAX_SPEED);
				rightMotor->setVelocity(MAX_SPEED/1.5);
				state = States::RUNNING;
			}
			break;
		}

		// printf("time:%f ir0:%f ir1:%f\n", robot->getTime(), ir0Val, ir1Val);
	};

	// Enter here exit cleanup code.
	delete robot;
	return 0;
}
