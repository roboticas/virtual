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

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define MAX_SPEED 6.28

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor* rightMotor = robot->getMotor("right wheel motor");
  
  DistanceSensor *ir0 = robot->getDistanceSensor("ir0");
  ir0->enable(timeStep);
  DistanceSensor* ir1 = robot->getDistanceSensor("ir1");
  ir1->enable(timeStep);

  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  leftMotor->setVelocity(MAX_SPEED);
  rightMotor->setVelocity(MAX_SPEED);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller

  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    double ir0Val = ir0->getValue();
    double ir1Val = ir1->getValue();


    if ((robot->getTime() > 1) && (ir0Val > 30)) {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
//        printf("Reversing : ir0:%f ir1:%f\n", ir0Val, ir1Val);
    }
      
//    printf("time:%f ir0:%f ir1:%f\n", robot->getTime(), ir0Val, ir1Val);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
