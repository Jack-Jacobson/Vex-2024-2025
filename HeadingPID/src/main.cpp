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
#include <cmath>

using namespace vex;
competition Competition;

vex::brain Brain;
vex::controller Controller;

motor frontLeftDrive = motor(PORT1, ratio18_1, false);
motor frontRightDrive = motor(PORT10, ratio18_1, true);
motor backLeftDrive = motor(PORT11, ratio18_1, false);
motor backRightDrive = motor(PORT20, ratio18_1, true);

motor_group leftDrive = motor_group(frontLeftDrive, backLeftDrive);
motor_group rightDrive = motor_group(frontRightDrive, backRightDrive);

int leftVel = 0, rightVel = 0, driveCompleted = 0;
bool temp = false, turnCompleted = false, functionRunning;
double leftDistance, turnSpeed, rightDistance, currentHeading, distanceToTarget, headingToTarget, targetHeading, p, i, d, error, pGain, leftVel2, rightVel2, LDT, RDT, PLDT, PRDT, DT, leftDistanceMM, rightDistanceMM, dXLoc, dYLoc, XLoc, YLoc;
double wheelDiameter = 101.6;
double wheelCircumference = 3.14159265358979323846 * wheelDiameter;

void pre_auton(void) {}

void UI() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
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
    Brain.Screen.print(temp ? "true" : "false");
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
    else if (headingDegrees > 360) headingDegrees -= 360;
    double relativeHeading = headingDegrees - currentHeading;
    if (relativeHeading > 180) relativeHeading -= 360;
    else if (relativeHeading < -180) relativeHeading += 360;
    return relativeHeading;
}

void updateCurrentHeading() {
    currentHeading = (rightDistance - leftDistance) / 350.8 * 57.29577951;
    currentHeading = currentHeading * -1;
    if (currentHeading >= 360) currentHeading -= 360;
    else if (currentHeading < 0) currentHeading += 360;
}

void updatePID() {
    leftDistance = ((frontLeftDrive.position(degrees) + backLeftDrive.position(degrees)) / 2.0) * (wheelCircumference / 360.0);
    rightDistance = ((frontRightDrive.position(degrees) + backRightDrive.position(degrees)) / 2.0) * (wheelCircumference / 360.0);
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
    else if (targetHeading >= 360) targetHeading -= 360;
    double error = targetHeading - currentHeading;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    while (fabs(error) > 2) {
        turnSpeed = error * 0.3;
        if (turnSpeed > 50) turnSpeed = 50;
        if (turnSpeed < -50) turnSpeed = -50;
        leftDrive.spin(forward, fabs(turnSpeed), percent);
        rightDrive.spin(reverse, fabs(turnSpeed), percent);
        error = targetHeading - currentHeading;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;
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
        return;
    }
}

void drivePath() {
    if (driveCompleted == 0) driveToPoint(0, 1200);
    else driveToPoint(0, 0);
}

void autonomous(void) {}

void usercontrol(void) {
    while (1) {
        if (Controller.ButtonR2.pressing()) {
            leftVel = Controller.Axis3.position() / 2;
            rightVel = Controller.Axis2.position() / 2;
            leftDrive.spin(forward, leftVel * 0.12, volt);
            rightDrive.spin(forward, rightVel * 0.12, volt);
        }
        if (Controller.ButtonUp.RELEASED) targetHeading += 10;
        if (Controller.ButtonDown.RELEASED) targetHeading -= 10;
        if (Controller.ButtonLeft.RELEASED) targetHeading = 0;
        if (Controller.ButtonRight.RELEASED) {
            frontLeftDrive.setPosition(0, degrees);
            frontRightDrive.setPosition(0, degrees);
            backLeftDrive.setPosition(0, degrees);
            backRightDrive.setPosition(0, degrees);
        }
        if (Controller.ButtonX.pressing()) drivePath();
        UI();
        updatePID();
        updateCurrentHeading();
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    usercontrol();
    pre_auton();
    while (true) wait(100, msec);
}
