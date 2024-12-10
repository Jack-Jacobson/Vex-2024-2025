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

    motor frontLeftDrive = motor(PORT15, ratio18_1, true);
    motor frontRightDrive = motor(PORT18, ratio6_1, false);
    motor middleLeftDrive = motor(PORT12, ratio6_1, true);
    motor middleRightDrive = motor(PORT17, ratio6_1, false);
    motor backLeftDrive = motor(PORT14, ratio18_1, true);
    motor backRightDrive = motor(PORT19, ratio6_1, false);
    motor intake = motor(PORT1, ratio6_1, true);

    digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);

    motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);
    motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

    int leftVel = 0, rightVel = 0, driveCompleted = 0, touchX, touchY, currentAuton = 2;

    bool temp = false, slowSpeed = false, turnCompleted = false, functionRunning, mogoClamped = false;
    double leftDistance, turnSpeed, rightDistance, currentHeading = 0, distanceToTarget, headingToTarget, targetHeading, startX = 0, startY = 0;
    double initialHeading = 90;
    double p, i, d, error, pGain, leftVel2, rightVel2, LDT, RDT, PLDT, PRDT, DT, leftDistanceMM, rightDistanceMM, dXLoc, dYLoc, XLoc = startX, YLoc = startY;
    double wheelDiameter = 69.85;
    double wheelbase = 304.8;
    double wheelCircumference = 3.14159265358979323846 * wheelDiameter;

    std::string screen = "main";    

    double normalizeHeading(double heading) {
        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;
        return heading;
    }

    void drawButton(int x, int y, int w, int h, std::string t, std::string destination) {
        Brain.Screen.setFillColor(black);   
        Brain.Screen.setPenColor(white);
        Brain.Screen.drawRectangle(x, y, w, h);
        
        int textWidth = Brain.Screen.getStringWidth(t.c_str());
        int textHeight = Brain.Screen.getStringHeight(t.c_str());
        int textX = x + (w - textWidth) / 2;
        int textY = y + h/2;
        Brain.Screen.printAt(textX, textY, false, t.c_str());
        
        if (Brain.Screen.pressing()) {
            int touchX = Brain.Screen.xPosition();
            int touchY = Brain.Screen.yPosition();

            if (touchX >= x && touchX <= x + w && touchY >= y && touchY <= y + h) {
                screen = destination; 
                Brain.Screen.clearScreen(); 
                return;
            }
        }
    }


    void UI() {

        if(screen == "main"){

        Brain.Screen.clearScreen();
        drawButton(0, 0, 240, 120, "PID Troubleshoot UI", "PIDUI" );
        drawButton(240, 0, 239, 120, "Motor Temperatures", "temps");
        drawButton(0, 120, 240, 119, "Other Troubleshooting UI", "troubleshootUI");   
        //drawButton(240, 120, 240, 119, "Auton Select", "atuon");     
        Brain.Screen.render();

        }
        else if(screen == "PIDUI"){
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor(black);
        Brain.Screen.clearScreen(); 
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
        Brain.Screen.print("distanceToTarget: ");
        Brain.Screen.print(distanceToTarget);
        Brain.Screen.newLine();
        Brain.Screen.print("DrivingToPoint?: ");
        if(functionRunning) Brain.Screen.print("True");
        else Brain.Screen.print("False");
        Brain.Screen.newLine();
        Brain.Screen.print("Current Scene: ");
        Brain.Screen.print("%s", screen.c_str());
        drawButton(429, 205, 50, 35, "Back", "main");
        Brain.Screen.render();
        }
        else if(screen == "temps"){
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor(black);
        Brain.Screen.clearScreen();
        Brain.Screen.print("Motor Temps (C): ");
        Brain.Screen.newLine();
        Brain.Screen.print("frontLeft): ");
        Brain.Screen.print(frontLeftDrive.temperature(celsius));
        Brain.Screen.print(",    middleLeft: ");
        Brain.Screen.print(middleLeftDrive.temperature(celsius));
        Brain.Screen.newLine();
        Brain.Screen.print("backLeft: ");
        Brain.Screen.print(backLeftDrive.temperature(celsius));
        Brain.Screen.print(",    frontRight: ");
        Brain.Screen.print(frontRightDrive.temperature(celsius));
        Brain.Screen.newLine();
        Brain.Screen.print("middleRight: ");
        Brain.Screen.print(middleRightDrive.temperature(celsius));
        Brain.Screen.print(",    backRight: ");
        Brain.Screen.print(backRightDrive.temperature(celsius));
        Brain.Screen.newLine();
        Brain.Screen.print("intake: ");
        Brain.Screen.print(intake.temperature(celsius));
        Brain.Screen.print(",    N/A: ");
        Brain.Screen.print(backRightDrive.temperature(celsius));
        drawButton(429, 205, 50, 35, "Back", "main");
        Brain.Screen.render();
        }
    else if(screen == "troubleshootUI"){
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.setFillColor(black);
        Brain.Screen.setPenColor(white);
        Brain.Screen.print("Left Y-Axis (3) Pos: ");
        Brain.Screen.print(Controller.Axis3.position());
        Brain.Screen.newLine();
        Brain.Screen.print("Right Y-Axis (2) Pos: ");
        Brain.Screen.print(Controller.Axis2.position());
        Brain.Screen.newLine();
        Brain.Screen.print("Actual leftDriveVelocity (percent): ");
        Brain.Screen.print(leftDrive.velocity(percent));
        Brain.Screen.newLine();
        Brain.Screen.print("Actual RightDriveVelocity (percent): ");
        Brain.Screen.print(rightDrive.velocity(percent));
        Brain.Screen.newLine();
        Brain.Screen.print("Touch Location (X,Y): ");
        Brain.Screen.print(touchX);
        Brain.Screen.print(", ");
        Brain.Screen.print(touchY);
        Brain.Screen.newLine();
        Brain.Screen.print("Current Scene: ");
        Brain.Screen.print("%s", screen.c_str());
            drawButton(429, 205, 50, 35, "Back", "main");
        Brain.Screen.render();
        }
    else{

        Brain.Screen.clearScreen();
        Brain.Screen.setPenColor(white);
        Brain.Screen.setCursor(1,1);
        Brain.Screen.setFillColor(black);
        Brain.Screen.print("Current Auton: ");
        if(currentAuton == 0){

            Brain.Screen.setPenColor(yellow);
            Brain.Screen.print("NONE SELECTED!!");

        }
        else if(currentAuton == 1){

            Brain.Screen.setPenColor(red);
            Brain.Screen.print("Red ALliance, High Stake Side");

        }
        else{

            Brain.Screen.setPenColor(blue);
            Brain.Screen.print("Blue Alliance, High Stake Side");

        }
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(0, 20, 240, 140);
        Brain.Screen.setFillColor(blue);
        Brain.Screen.drawRectangle(240, 20, 239, 140);
        Brain.Screen.setFillColor(yellow);
        Brain.Screen.drawRectangle(0, 160, 479, 79);
        if (Brain.Screen.pressing()) {
    int touchX = Brain.Screen.xPosition();
    int touchY = Brain.Screen.yPosition();
    if (touchY < 160) {
        if (touchX < 240) currentAuton = 1;  
        else currentAuton = 2;              
    } else {
        currentAuton = 0;                   
}
        

        drawButton(429, 205, 50, 35, "Back", "main");
        Brain.Screen.render();
        }



    }
        vex::task::sleep(100);
    }


    double calculateHeading(double targetXLoc, double targetYLoc) {
        double deltaX = targetXLoc - XLoc;
        double deltaY = targetYLoc - YLoc;
        double headingRadians = atan2(deltaY, deltaX);
        double headingDegrees = headingRadians * (180.0 / 3.14159265358979323846);

        headingDegrees = normalizeHeading(headingDegrees); 
        double relativeHeading = normalizeHeading(headingDegrees - (currentHeading - initialHeading));
        return relativeHeading;
    }



void updateCurrentHeading() {
    double deltaHeading = (rightDistance - leftDistance) / 304.8; 
    currentHeading = initialHeading - (deltaHeading * (180.0 / M_PI)); 
    currentHeading = normalizeHeading(currentHeading);
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
        targetHeading = normalizeHeading(targetHeading);
        double error = normalizeHeading(targetHeading - (currentHeading - initialHeading));
        
        int maxTurnSpeed = 50;
        int minTurnSpeed = 20;

        while (fabs(error) > 2) { 
            turnSpeed = error * 0.35;

            if (turnSpeed > maxTurnSpeed) turnSpeed = maxTurnSpeed;
            else if (turnSpeed < -maxTurnSpeed) turnSpeed = -maxTurnSpeed;
            else if (turnSpeed > 0 && turnSpeed < minTurnSpeed) turnSpeed = minTurnSpeed;
            else if (turnSpeed < 0 && turnSpeed > -minTurnSpeed) turnSpeed = -minTurnSpeed;
    if (fabs(error) < 2 && fabs(turnSpeed) < 5) {
        leftDrive.stop();
        rightDrive.stop();
        return;
    }



            leftDrive.spin(forward, turnSpeed, percent);
            rightDrive.spin(reverse, turnSpeed, percent);

            updatePID();
            updateCurrentHeading();
            UI();
            
            error = normalizeHeading(targetHeading - (currentHeading - initialHeading)); 
        }
    }


    
void driveToPoint(double targetXLoc, double targetYLoc) {
    headingToTarget = calculateHeading(targetXLoc, targetYLoc);
    double deltaX = targetXLoc - XLoc;
    double deltaY = targetYLoc - YLoc;
    distanceToTarget = sqrt((deltaX * deltaX) + (deltaY * deltaY));

    // Turn to the initial heading towards the target
    turnToHeading(headingToTarget);

    double maxSpeed = 50;   // Maximum driving speed
    double minSpeed = 15;   // Minimum driving speed for fine adjustments
    double slowDownDistance = 200; // Distance threshold to begin slowing down
    functionRunning = true;

    while (functionRunning) {
        updatePID();
        updateCurrentHeading();

        // Recalculate position, heading, and distance to the target
        deltaX = targetXLoc - XLoc;
        deltaY = targetYLoc - YLoc;
        distanceToTarget = sqrt((deltaX * deltaX) + (deltaY * deltaY));
        headingToTarget = calculateHeading(targetXLoc, targetYLoc);

        if (distanceToTarget < 40) {
            leftDrive.stop();
            rightDrive.stop();
            functionRunning = false;
            driveCompleted++;
            break;
        }

        double driveSpeed = maxSpeed;
        if (distanceToTarget < slowDownDistance) {
            driveSpeed = std::max(minSpeed, maxSpeed * (distanceToTarget / slowDownDistance));
        }

        double headingError = normalizeHeading(headingToTarget - currentHeading);
        double correctionFactor = headingError * 0.2; // Reduce gain to avoid overcorrection

        double leftSpeed = driveSpeed - correctionFactor;
        double rightSpeed = driveSpeed + correctionFactor;

        // Ensure motors don't exceed speed limits or reverse too strongly
        if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
        else if (leftSpeed < minSpeed) leftSpeed = minSpeed;

        if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
        else if (rightSpeed < minSpeed) rightSpeed = minSpeed;

        // Apply speeds to motors
        leftDrive.setVelocity(leftSpeed, percent);
        rightDrive.setVelocity(rightSpeed, percent);
        leftDrive.spin(forward);
        rightDrive.spin(forward);
    }
}


void drivePath(double points[][2], int numPoints) {
    for (int i = 0; i < numPoints; i++) {
        double targetX = points[i][0];
        double targetY = points[i][1];
        driveToPoint(targetX, targetY);
        wait(50, msec); 
    }

    updateCurrentHeading();
    updatePID();
    UI();
    }

void driveForward(int degreeNum){

leftDrive.spinFor(degreeNum, degrees, false);
   rightDrive.spinFor(degreeNum, degrees);

}
void driveReverse(int degreeNum){

    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);

}

void setVelocity(int velocity){

    leftDrive.setVelocity(velocity, percent);
    rightDrive.setVelocity(velocity, percent);

}

void turn(int direction, int degreeNum){

    if(direction == 0){

        leftDrive.spinFor(reverse, degreeNum, degrees, false);
        rightDrive.spinFor(degreeNum, degrees);

    }
    else{

        leftDrive.spinFor( degreeNum, degrees, false);
        rightDrive.spinFor(reverse, degreeNum, degrees);
    }

}

    void pre_auton() {

        while(true){

            UI();

        }

    }

    void autonomous() {
    if(currentAuton == 2){
        intake.setVelocity(50, percent);
    setVelocity(50);
   driveReverse(1400);
   setVelocity(20);
   driveReverse(1140);
   mogoClamp.set(true);
   mogoClamped = true;
   intake.spinFor(800, degrees, false);
   turn(1, 500);
   mogoClamp.set(false);
   mogoClamped = false;
   setVelocity(30);
   driveForward(700);
   wait(1, seconds);
   intake.stop(brake);
    intake.spinFor(800, degrees, false);
   driveForward(420);
  setVelocity(15);
   turn(0, 570);
   driveReverse(730);
   mogoClamp.set(true);
   mogoClamped = true;
   intake.spin(forward);
   wait(1, seconds);
   setVelocity(40);
   driveForward(800);
   mogoClamp.set(false);
   mogoClamped = false;
   turn(0, 550);
   driveForward(4000);
    }
    else if(currentAuton == 1){
       setVelocity(50);
   driveReverse(1400);
   setVelocity(20);
   driveReverse(1140);
   mogoClamp.set(true);
   mogoClamped = true;
   intake.setVelocity(100, percent);
   wait(1, seconds);
   intake.spin(forward);
   wait(1, seconds);
   turn(0, 500);
   mogoClamp.set(false);
   mogoClamped = false;
   setVelocity(30);
   driveForward(700);
   wait(1, seconds);
   intake.stop(brake);
   intake.spinFor(reverse, 1000, degrees, false);
   driveForward(420);
      setVelocity(15);
   turn(1, 570);
   driveReverse(730);
   mogoClamp.set(true);
   mogoClamped = true;
   intake.spin(forward);
   wait(2, seconds);
   setVelocity(40);
   driveForward(800);
      mogoClamp.set(false);
   mogoClamped = false;
   turn(1, 550);
   driveForward(4000);

    }
    else {Brain.Screen.print("NO AUTON SELECTED");}
    }

    void usercontrol() {   
        while (1) {
            int leftSpeed = (Controller.Axis3.position() + Controller.Axis1.position())*0.12;
            int rightSpeed = (Controller.Axis3.position() - Controller.Axis1.position())* 0.12;

            if(leftSpeed>100){

                leftSpeed = 100;

            }
            if(rightSpeed>100){

                rightSpeed = 100;

            }


            leftDrive.spin(forward, leftSpeed, volt);
            rightDrive.spin(forward, rightSpeed, volt);

            if (Controller.ButtonR2.pressing()) intake.spin(forward, 12, volt);
            else if (Controller.ButtonR1.pressing()) intake.spin(reverse, 12, volt);
            else intake.stop();


            double path1[][2] = {

                {1200, 0},
                {0, 0},
                {1200, 0},
                {0,0 }

            };

        if (Controller.ButtonY.pressing()) {
        /** 
    double path1[][2] = {
        {1200, 0},
        {1200, 600},
        {600, 600},
        {0, 0}
    };
    drivePath(path1, 4);
    */


}


            if (Controller.ButtonL2.PRESSED) {
                if(mogoClamped == false) {
                    mogoClamped = true;
                    mogoClamp.set(true);
                }
                else {
                    mogoClamped = false;
                    mogoClamp.set(false);
                }
            }

            wait(20, msec);
            UI(); 
            updatePID();
            updateCurrentHeading();
        }
    }

    int main() {
        Competition.autonomous(autonomous);
        Competition.drivercontrol(usercontrol);
        pre_auton();
        while (true) wait(100, msec);
    }
