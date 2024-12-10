/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jokin                                                     */
/*    Created:      11/4/2024, 5:47:54 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

 #include "vex.h"
#include <iostream>
#include <cstring>
#include <cmath>
using namespace vex;

competition Competition;

vex::brain Brain;
vex::controller Controller;

motor frontLeftDrive = motor(PORT14, ratio18_1, true);
motor frontRightDrive = motor(PORT18, ratio6_1, false);
motor middleLeftDrive = motor(PORT12, ratio6_1, true);
motor middleRightDrive = motor(PORT20, ratio6_1, false);
motor backLeftDrive = motor(PORT15, ratio18_1, true);
motor backRightDrive = motor(PORT17, ratio6_1, false);
motor intake = motor(PORT1, ratio6_1, true);

digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);

motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

int leftVel = 0, rightVel = 0, driveCompleted = 0;

bool temp = false, slowSpeed = false, turnCompleted = false, functionRunning, mogoClamped = false;
double leftDistance, turnSpeed, rightDistance, currentHeading, distanceToTarget, headingToTarget, targetHeading;
double p, i, d, error, pGain, leftVel2, rightVel2, LDT, RDT, PLDT, PRDT, DT, leftDistanceMM, rightDistanceMM, dXLoc, dYLoc, XLoc, YLoc;
double wheelDiameter = 69.85;
double wheelCircumference = 3.14159265358979323846 * wheelDiameter;

void UI() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Current heading: ");
    Brain.Screen.print(currentHeading);
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
    Brain.Screen.newLine();
    Brain.Screen.print("Target heading: ");
    Brain.Screen.print(headingToTarget);
    Brain.Screen.newLine();
    Brain.Screen.print("Temp value: ");
    Brain.Screen.print(temp);
    Brain.Screen.newLine();
    Brain.Screen.print("Error in Heading: ");
    Brain.Screen.print(fabs(currentHeading - headingToTarget));
    Brain.Screen.newLine();
    Brain.Screen.print("driveNumber: ");
    Brain.Screen.print(driveCompleted);
    Brain.Screen.newLine();
    Brain.Screen.print("DrivingToPoint?: ");
    Brain.Screen.print(functionRunning);
    Brain.Screen.newLine();
    Brain.Screen.render();
    vex::task::sleep(100);
}

double calculateHeading(double targetXLoc, double targetYLoc) {
    double deltaX = targetXLoc - XLoc;
    double deltaY = targetYLoc - YLoc;
    double headingRadians = atan2(deltaY, deltaX);
    double headingDegrees = headingRadians * (180.0 / 3.14159265358979323846);
    if (headingDegrees < 0) headingDegrees += 360;
    if (headingDegrees > 360) headingDegrees -= 360;
    double relativeHeading = headingDegrees - currentHeading;
    if (relativeHeading > 180) relativeHeading -= 360;
    if (relativeHeading < -180) relativeHeading += 360;
    return relativeHeading;
}

void updateCurrentHeading() {
    currentHeading = (rightDistance - leftDistance) / 350.8 * 57.29577951;
    currentHeading *= -1;
    if (currentHeading >= 360) currentHeading -= 360;
    if (currentHeading < 0) currentHeading += 360;
}

void updatePID() {
    leftDistance = ((frontLeftDrive.position(degrees) + backLeftDrive.position(degrees) + middleLeftDrive.position(degrees)) / 3.0) * (wheelCircumference / 360.0);
    rightDistance = ((frontRightDrive.position(degrees) + backRightDrive.position(degrees) + middleRightDrive.position(degrees)) / 3.0) * (wheelCircumference / 360.0);
    leftDistanceMM = leftDistance * 0.887;
    rightDistanceMM = rightDistance * 0.887;
    LDT = leftDistanceMM - PLDT;
    RDT = rightDistanceMM - PRDT;
    PLDT = leftDistanceMM;
    PRDT = rightDistanceMM;
    updateCurrentHeading();
    dXLoc = ((LDT + RDT) / 2) * cos(currentHeading * (3.14159265358979323846 / 180));
    dYLoc = ((LDT + RDT) / 2) * sin(currentHeading * (3.14159265358979323846 / 180));
    XLoc += dXLoc;
    YLoc += dYLoc;
    vex::task::sleep(10);
}

void turnToHeading(double targetHeading) {
    if (targetHeading < 0) targetHeading += 360;
    if (targetHeading >= 360) targetHeading -= 360;
    double error = targetHeading - currentHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    while (fabs(error) > 2) {
        turnSpeed = error * 0.3;
        if (turnSpeed > 50) turnSpeed = 50;
        if (turnSpeed < -50) turnSpeed = -50;
        leftDrive.spin(forward, fabs(turnSpeed), percent);
        rightDrive.spin(reverse, fabs(turnSpeed), percent);
        error = targetHeading - currentHeading;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        updatePID();
        UI();
    }
    if (fabs(error) <= 2) turnCompleted = true;
    leftDrive.stop();
    rightDrive.stop();
}

void driveToPoint(double targetXLoc, double targetYLoc) {
    double error = fabs(currentHeading - headingToTarget);
    updateCurrentHeading();
    headingToTarget = calculateHeading(targetXLoc, targetYLoc);
    double deltaX = targetXLoc - XLoc;
    double deltaY = targetYLoc - YLoc;
    distanceToTarget = sqrt((deltaX * deltaX) + (deltaY * deltaY));
    double degreesToTravel = (distanceToTarget / wheelCircumference) * 360;
    double driveSpeed = 50;
    turnToHeading(headingToTarget);
    temp = true;
    leftDrive.setVelocity(driveSpeed, percent);
    rightDrive.setVelocity(driveSpeed, percent);
    leftDrive.spinFor(forward, degreesToTravel, degrees, false);
    rightDrive.spinFor(forward, degreesToTravel, degrees);
    functionRunning = true;
    if (distanceToTarget < 10) {
        functionRunning = false;
        leftDrive.stop();
        driveCompleted++;
        rightDrive.stop();
    }
}

void pre_auton() {}

void autonomous() {
    leftDrive.setVelocity(30, percent);
    rightDrive.setVelocity(30, percent);
   leftDrive.spinFor(forward, 2500, degrees, false);
   rightDrive.spinFor(forward, 2500,  degrees);
}

void usercontrol() {
    while (1) {
        double leftSpeed = Controller.Axis3.position();
        double rightSpeed = Controller.Axis2.position();
        if (slowSpeed == false){ 
            leftDrive.spin(forward, leftSpeed * 0.12, volt); 
        rightDrive.spin(forward, rightSpeed * 0.12, volt); }
        else { 
            leftDrive.spin(forward, leftSpeed*0.06, volt); 
        rightDrive.spin(forward, rightSpeed*0.06, volt);
        }
        if(Controller.ButtonL1.PRESSED){
            if (slowSpeed) slowSpeed = false;
            else slowSpeed = true;
        }
        if (Controller.ButtonR2.pressing()) intake.spin(forward, 12, volt);
        else if (Controller.ButtonR1.pressing()) intake.spin(reverse, 12, volt);
        else intake.stop();
        if (Controller.ButtonL2.PRESSED) {
            if(mogoClamped == false){

                mogoClamped = true;
                mogoClamp.set(true);

            }
            else{

                mogoClamped = false;
                mogoClamp.set(false);


            }
       
        }
        wait(20, msec);
        UI(); 
 
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();
    while (true) wait(100, msec);
}
