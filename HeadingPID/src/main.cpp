/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jack Jacobson                                             */
/*    Created:      9/4/2024, 7:00:48 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include "stdarg.h"
#include <cstring>
#include <string.h>

using namespace vex;
competition Competition;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::controller Controller;

motor frontLeftDrive = motor(PORT1, ratio18_1, false);
motor frontRightDrive = motor(PORT10, ratio18_1, true);
motor backLeftDrive = motor(PORT11, ratio18_1, false);
motor backRightDrive = motor(PORT20, ratio18_1, true);

motor_group leftDrive = motor_group(frontLeftDrive, backLeftDrive);
motor_group rightDrive = motor_group(frontRightDrive, backRightDrive);

int leftVel = 0, rightVel = 0;

bool manualControl = false, spinnySpin = false; 
double leftDistance, rightDistance, currentHeading, targetHeading, p, i, d, error, pGain, leftVel2, rightVel2, LDT, RDT, PLDT, PRDT, DT, leftDistanceMM, rightDistanceMM, dXLoc, dYLoc, XLoc, YLoc; 


void pre_auton(void) {

}

void PID(){

    leftDistance = (frontLeftDrive.position(degrees) + backLeftDrive.position(degrees)) / 2.0;
    rightDistance = (frontRightDrive.position(degrees) + backRightDrive.position(degrees)) / 2.0;
    leftDistanceMM = leftDistance * 0.887;
    rightDistanceMM = rightDistance * 0.887;
    LDT = leftDistanceMM - PLDT;  
    RDT = rightDistanceMM - PRDT;  
    PLDT = leftDistanceMM;
    PRDT = rightDistanceMM;


if(Controller.ButtonA.pressing()){

if(p >50){

p = 50;

}
else if(p < -50){

p = -50;

}

leftVel2 = p;
rightVel2 = p*-1;

if(leftVel2 >0 && leftVel2 < 1){

leftVel2 = 0;

}
/** 
if(rightVel2 > -1 && rightVel2 < 0){

rightVel2 = 0;

}
*/

leftDrive.setVelocity(leftVel2, percent);
rightDrive.setVelocity(rightVel2, percent);

leftDrive.spin(forward);
rightDrive.spin(forward);

dXLoc = ((LDT+RDT)/2)*cos(currentHeading*(3.14/180));
dYLoc = ((LDT+RDT)/2)*sin(currentHeading*(3.14/180));
XLoc+=dXLoc;
YLoc+=dYLoc;

vex::task::sleep(10);

}




}

void autonomous(void) {
  
}

void UI(void){

Brain.Screen.clearScreen();
Brain.Screen.setCursor(1,1);
Brain.Screen.print("Current heading: ");
Brain.Screen.print(currentHeading);
Brain.Screen.setCursor(2,1);
Brain.Screen.print("Target heading: ");
Brain.Screen.print(targetHeading);
Brain.Screen.newLine();
Brain.Screen.print(leftVel2);
Brain.Screen.newLine();
Brain.Screen.print(rightVel2);
Brain.Screen.newLine();
Brain.Screen.print("PLDT: ");
Brain.Screen.print(PLDT);
Brain.Screen.newLine();
Brain.Screen.print("PRDT: ");
Brain.Screen.print(PRDT);
Brain.Screen.newLine();
Brain.Screen.print("LDT: ");
Brain.Screen.print(LDT);
Brain.Screen.newLine();
Brain.Screen.print("RDT: ");
Brain.Screen.print(RDT);
Brain.Screen.newLine();
Brain.Screen.print("Left Distance (mm): ");
Brain.Screen.print(leftDistanceMM);
Brain.Screen.newLine();
Brain.Screen.print("Right Distance (mm): ");
Brain.Screen.print(rightDistanceMM);
Brain.Screen.newLine();
Brain.Screen.print("XLoc: ");
Brain.Screen.print(XLoc);
Brain.Screen.newLine();
Brain.Screen.print("YLoc: ");
Brain.Screen.print(YLoc);
vex::task::sleep(100);

}

void usercontrol(void) {

  while (1){
if(Controller.ButtonR2.pressing()){

leftVel = Controller.Axis3.position();
  rightVel = Controller.Axis2.position();

  leftDrive.spin(forward, leftVel*0.12, volt);
 rightDrive.spin(forward, rightVel*0.12, volt);

}

  if(Controller.ButtonUp.RELEASED){

    targetHeading+=10; 

  }
  if(Controller.ButtonDown.RELEASED){

    targetHeading-=10;

  }
  if(Controller.ButtonLeft.RELEASED){

    targetHeading = 0;

  }
  if(Controller.ButtonRight.RELEASED){

    frontLeftDrive.setPosition(0, degrees);
    frontRightDrive.setPosition(0, degrees);
    backLeftDrive.setPosition(0, degrees);
    backRightDrive.setPosition(0, degrees);

  }
  vex::thread uiThread(UI);
  vex::thread PIDThread(PID);

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

