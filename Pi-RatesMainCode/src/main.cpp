/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jack Jacobson                                             */
/*    Created:      9/4/2024, 7:00:48 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;
competition Competition;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::controller Controller;

motor frontLeftDrive = motor(PORT16, ratio6_1, true);
motor middleLeftDrive = motor(PORT11 ,ratio6_1, true);
motor backLeftDrive = motor(PORT14, ratio6_1, true);
motor frontRightDrive = motor(PORT3, ratio6_1, false);
motor middleRightDrive = motor(PORT4, ratio6_1, false);
motor backRightDrive = motor (PORT2, ratio6_1, false);

motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

void pre_auton(void) {

}

void autonomous(void) {
  
}



void usercontrol(void) {

  while (1){
     
   leftDrive.setVelocity(Controller.Axis3.position(), percent);
   rightDrive.setVelocity(Controller.Axis2.position() , percent);

   leftDrive.spin(forward);
   rightDrive.spin(forward);

  }



  }




int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  usercontrol(); 

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

