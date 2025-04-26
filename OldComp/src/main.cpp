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

        // Motor and sensor declarations
        motor frontLeftDrive = motor(PORT15, ratio18_1, true);
        motor frontRightDrive = motor(PORT18, ratio6_1, false);
        motor middleLeftDrive = motor(PORT12, ratio6_1, true);
        motor middleRightDrive = motor(PORT17, ratio6_1, false);
        motor backLeftDrive = motor(PORT14, ratio18_1, true);
        motor backRightDrive = motor(PORT19, ratio6_1, false);
        motor intake = motor(PORT1, ratio6_1, true);
       // motor intake = motor(PORT1, ratio6_1, true);
        motor LB = motor(PORT2, ratio6_1, false);

        rotation LBRot = rotation(PORT3, true);

        digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);
        digital_out doinker = digital_out(Brain.ThreeWirePort.D);

        inertial inertialSensor = inertial(PORT12);

        motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);
        motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

        // Variable declarations
        int leftVel = 0, rightVel = 0, fwdSpeed; // Velocity of each side of the robot
        int touchX, touchY; // Variables for screen touch coordinates
        int currentAuton = 2; // Selected autonomous routine
        bool mogoClamped = false;
        bool doinked = false; // State of the mogo clamp
        bool fastTurn = false; // Toggle for fast turning
        bool intertialCalibrated = false;
        double kP = 0.2, integral = 0.0, kI = 0.005, kD = 0.00, previousError = 0, LeftPIDTurningOutput = 0, RightPIDTurningOutput = 0; // PID constants
        int counter = 0;
        int counter2 = 0;
        double targetHeading, threshold = 2, tkP = 0.16, tkI = 0.0, tkD = 0.09; // PID variables
        double leftPIDOutput, rightPIDOutput; // PID output variables
        double power = 0.0, LBPower = 0.0; // Power variable for motors
        int currentHeading = 0; // Current heading from the inertial sensor

        double totalDegrees = 0.0;
        double prevLeft = 0.0;
        double prevRight = 0.0;

        //double fwdSpeed = 60; // Forward velocity

        bool tasksCompleted[100] = {
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false
        };
        


        std::string screen = "main"; // Current screen displayed on the brain

        int degreeTrackerTask() {
            while (true) {
                  
                double left = leftDrive.position(degrees);
                double right = rightDrive.position(degrees);

                double deltaLeft = left - prevLeft;
                double deltaRight = right - prevRight;
                double avgDeltaDeg = (deltaLeft + deltaRight) / 2.0;

                totalDegrees += abs(avgDeltaDeg);

                prevLeft = left;
                prevRight = right;
                
                task::sleep(20);
            }
            return 0;
        }
        task degreeTask(degreeTrackerTask);



        #pragma region // Functions for UI on Brain and Controller
        // Function to draw a button on the Brain screen
        void drawButton(int x, int y, int w, int h, std::string t, std::string destination, std::string Screen) {
            Brain.Screen.setFillColor(black);
            Brain.Screen.setPenColor(white);
            Brain.Screen.drawRectangle(x, y, w, h);

            int textWidth = Brain.Screen.getStringWidth(t.c_str());
            int textHeight = Brain.Screen.getStringHeight(t.c_str());
            int textX = x + (w - textWidth) / 2;
            int textY = y + h / 2;
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

        // Function to handle the UI on the Brain screen
        void UI() {
            if (screen == "main") {
                Brain.Screen.clearScreen();
                drawButton(0, 0, 240, 120, "PID Troubleshoot UI", "PIDUI", "main");
                drawButton(240, 0, 239, 120, "Motor Temperatures", "temps", "main");
                drawButton(0, 0, 240, 119, "Other Troubleshooting UI", "troubleshootUI", "main");
                drawButton(0, 120, 240, 119, "Auton Select", "atuon", "main");
                Brain.Screen.render();
            } else if (screen == "PIDUI") {
                #pragma region // PID UI, displays all variable associated with PID, currently disable due to auton selector bugs.
                
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.setPenColor(white);
                Brain.Screen.setFillColor(black);
                Brain.Screen.clearScreen();
                Brain.Screen.print("Current heading (inertial): ");
                Brain.Screen.print(inertialSensor.heading(degrees));
                drawButton(429, 205, 50, 35, "Back", "main", "PIDUI");
                Brain.Screen.render();
                
                #pragma endregion
            } else if (screen == "temps") {
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
                Brain.Screen.print(",    Lady Brown: ");
               Brain.Screen.print(LB.temperature(celsius));
                drawButton(429, 205, 50, 35, "Back", "main", "temps");
                Brain.Screen.render();
            } else if (screen == "troubleshootUI") {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
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
                Brain.Screen.print("Lady Brown Location: ");
                Brain.Screen.print(LBRot.position(degrees));
                Brain.Screen.newLine();
                Brain.Screen.print("Current Scene: ");
                Brain.Screen.print("%s", screen.c_str());
                Brain.Screen.print("fastTurn: ");
                Brain.Screen.print(fastTurn);
                drawButton(429, 205, 50, 35, "Back", "main", "troubleshootUI");
                Brain.Screen.render();
            } else if (screen == "atuon") {
                printf("Current Auton: %.2f\n", currentAuton);
            
                if (Controller.ButtonUp.PRESSED) {
                    currentAuton++;
                }
                if (Controller.ButtonDown.PRESSED) {
                    currentAuton--;
                }
            
                if (currentAuton < 0) {
                    currentAuton = 4;
                }
                if (currentAuton > 4) {
                    currentAuton = 0;
                }
            
                // Update controller display regardless of touch
                Controller.Screen.clearLine(3);
                if (currentAuton == 0) {
                    Controller.Screen.print("NO AUTON");
                } else if (currentAuton == 1) {
                    Controller.Screen.print("Red Alliance, HS");
                } else if (currentAuton == 2) {
                    Controller.Screen.print("Blue Alliance, HS");
                } else if (currentAuton == 3) {
                    Controller.Screen.print("Red Alliance, NHS");
                } else if (currentAuton == 4) {
                    Controller.Screen.print("Blue Alliance, NHS");
                }
            
                Brain.Screen.clearLine(1);
                Brain.Screen.setPenColor(white);
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.setFillColor(black);
                Brain.Screen.print("Current Auton: ");
                if (currentAuton == 0) {
                    Brain.Screen.setPenColor(yellow);
                    Brain.Screen.print("NONE SELECTED!!");
                } else if (currentAuton == 1 || currentAuton == 3) {
                    Brain.Screen.setPenColor(red);
                    Brain.Screen.print("Red Alliance, ");
                    if (currentAuton == 1) {
                        Brain.Screen.print("High Stake Side");
                    } else {
                        Brain.Screen.print("Non-High Stake Side");
                    }
                } else {
                    Brain.Screen.setPenColor(blue);
                    Brain.Screen.print("Blue Alliance, ");
                    if (currentAuton == 2) {
                        Brain.Screen.print("High Stake Side");
                    } else if (currentAuton == 4) {
                        Brain.Screen.print("Non-High Stake Side");
                    }
                }
            
                Brain.Screen.setFillColor(red);
                Brain.Screen.drawRectangle(0, 20, 240, 70);
                Brain.Screen.setFillColor(blue);
                Brain.Screen.drawRectangle(240, 20, 239, 70);
                Brain.Screen.setFillColor(green);
                Brain.Screen.drawRectangle(0, 90, 240, 70);
                Brain.Screen.setFillColor(purple);
                Brain.Screen.drawRectangle(240, 90, 240, 70);
                Brain.Screen.setFillColor(yellow);
                Brain.Screen.drawRectangle(0, 160, 240, 79);
            
                if (Brain.Screen.pressing()) {
                    int touchX = Brain.Screen.xPosition();
                    int touchY = Brain.Screen.yPosition();
                    if (touchY < 90) {
                        if (touchX < 240 && currentAuton != 1) {
                            currentAuton = 1;
                        } else if (currentAuton != 2) {
                            currentAuton = 2;
                        }
                    } else if (touchY > 90 && touchY < 160) {
                        if (touchX < 240) {
                            currentAuton = 3;
                        } else {
                            currentAuton = 4;
                        }
                    } else if (touchX < 240 && currentAuton != 0) {
                        currentAuton = 0;
                    }
                }
            
                drawButton(240, 160, 239, 79, "Back", "main", "auton");
                Brain.Screen.render();
            }
        
            vex::task::sleep(100);
        }
        #pragma endregion

        #pragma region // PID Functions
        int turnToHeading(void) {
            // tkP = 0.25, tkI = 0.0, tkD = 0
            double previousError = 0;
            double integral = 0;

            while (true) {
                currentHeading = inertialSensor.heading(degrees);
                double error = targetHeading - currentHeading;

                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                // PID calculations
                double proportionOutput = tkP * error;
                integral += error;
                double integralOutput = integral * tkI;
                double derivative = error - previousError;
                double derivativeOutput = tkD * derivative;

                power = proportionOutput + integralOutput + derivativeOutput;

                if (power > 12) power = 12;
                if (power < -12) power = -12;
                if (power < 1 && power > 0 ) power = 0;
                if (power > -1 && power < 0) power = 0;
                

                //printf("Power: %.2f | Heading: %.2f | Error: %.2f | Previous Error: %.2f | P Gain: %.2f | I Gain: %.2f | D Gain: %.2f \n", power, currentHeading, error, previousError, proportionOutput, integralOutput, derivativeOutput);    

                backLeftDrive.setReversed(true);
                frontRightDrive.setReversed(false);
                leftDrive.spin(forward,fwdSpeed + Controller.Axis3.position() + Controller.Axis1.position(), volt);
                rightDrive.spin(forward, 0.5*( fwdSpeed + Controller.Axis3.position() - Controller.Axis1.position()), volt);

                previousError = error;

                

                vex::this_thread::sleep_for(10);
            }
                
        }
        int LBPID(void){

                double error = 26 - LBRot.position(degrees);

                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                // PID calculations
                double proportionOutput = kP * error;
                integral += error;
                double integralOutput = integral * kI;
                double derivative = error - previousError;
                double derivativeOutput = kD * derivative;

                LBPower = proportionOutput + integralOutput + derivativeOutput;

                if (power > 12) power = 12;
                if (power < -12) power = -12;
                if (power < 1 && power > 0 ) power = 0;
                if (power > -1 && power < 0) power = 0;


                previousError = error;

                printf("Power: %.2f | Pos: %.2f | Error: %.2f | Previous Error: %.2f | P Gain: %.2f | I Gain: %.2f | D Gain: %.2f \n", LBPower, LBRot.position(degrees), error, previousError, proportionOutput, integralOutput, derivativeOutput);
                return 0;
                
            

        }


        int UIUpdate(void) {
            while (true) {
                //printf("Current heading: %.2f | Target heading: %.2f | Left PID Output: %.2f | Right PID Output: %.2f\n", inertialSensor.heading(degrees), targetHeading, leftPIDOutput, rightPIDOutput);
                //printf("xPosition: %.2f | yPosition: %.2f | Left Drive Position: %.2f | Right Drive Position: %.2f\n", posX, posY, leftDrive.position(degrees), rightDrive.position(degrees));
                //printf("fwdSpeed: %.2f  |  PIDOutput: %.2f  |  currentHeading: %.2f |  distanceTravelled: %.2f\n", fwdSpeed, power, inertialSensor.heading(degrees), totalDegrees);
            //   printf("Distance Travelled: %.2f\n", totalDegrees);

               vex::this_thread::sleep_for(100);
            }
        }

        int checkEvents(void){

            while(true){
                /*
                if(totalDegrees>1000&&tasksCompleted[0] == false){

                    targetHeading = 90;
                    fwdSpeed = 25;
                    tasksCompleted[0] = true;

                }
                    */
            }

            vex::this_thread::sleep_for(10);

        }
            

        #pragma endregion
        

        // Function to run before autonomous
        void pre_auton() {
            while (true) {
            UI(); // Makes sure that you are able to use the auton selector and check motor temps while the controller is plugged in before auton.
            } // Constantly updates the UI so you can properly use it.
        } // Code running before auton; when the controller is plugged into field control before auton runs.

        // Autonomous routine

        // User control routine
        void usercontrol() {
            // double targetHeading = inertialSensor.heading(degrees); // Initial heading
            while (true) {       


        int leftSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        int rightSpeed = Controller.Axis3.position() - Controller.Axis1.position();
            
        leftDrive.spin(forward, 1*(leftSpeed*0.12), volt);
        rightDrive.spin(forward, 0.9*(rightSpeed*0.12), volt);
                
            /** 
                double leftSpeed = fwdSpeed*0.12 + leftPIDOutput;
                double rightSpeed = fwdSpeed*0.12 + rightPIDOutput;
                */
                // Spin the motors with the calculated spe  eds
                //leftDrive.spin(forward, fwdSpeed + leftPIDOutput, volt);
                //rightDrive.spin(forward, fwdSpeed + rightPIDOutput, volt);

                // printf("leftSpeed: %.2f | rightSpeed: %.2f | leftPIDOutput: %.2f | rightPIDOutput: %.2f\n", leftSpeed, rightSpeed, leftPIDOutput, rightPIDOutput);
                
                if(Controller.ButtonB.pressing()){
                    //printf("Current heading: %.2f\n", inertialSensor.heading(degrees));
                    //printf("Target heading: %.2f\n", targetHeading);
                    //printf("Left PID Output: %.2f\n", leftPIDOutput);
                    //printf("Right PID Output: %.2f\n", rightPIDOutput);
                    targetHeading = 90+inertialSensor.heading(degrees);
                }

                else{

                    //LB.setStopping(coast);

                }

                if(Controller.ButtonL2.RELEASED){

                    if(mogoClamped == false){
                        mogoClamp.set(true);
                        mogoClamped = true;
                    }
                    else{
                        mogoClamp.set(false);
                        mogoClamped = false;
                    }

                }
                if(Controller.ButtonUp.RELEASED){

                    if(doinked == false){
                        doinker.set(true);
                        doinked = true;
                    }
                    else{
                        doinker.set(false);
                        doinked = false;
                    }   

                }

                if(Controller.ButtonA.pressing()){
                    LBRot.setPosition(0, degrees);
                }

                if(Controller.ButtonX.pressing()) {
                    LB.spin(forward, 12, volt);
                    std::cout << "Spinning forward" << std::endl;
                } else if (Controller.ButtonB.pressing()) {
                    LB.spin(reverse, 12, volt);
                } else if (Controller.ButtonL1.pressing()) {
                    LBPID();
                    LB.spin(forward, LBPower, volt);
                } else {
                    LB.stop(); // Stop the motor when no related buttons are pressed
                }

                if(Controller.ButtonRight.pressing()){  
                  //  printf("Current heading: %.2f\n", inertialSensor.heading(degrees));
                }
                if (Controller.ButtonR2.pressing()) intake.spin(forward, 12, volt);
                else if (Controller.ButtonR1.pressing()) intake.spin(reverse, 12, volt);
                else intake.stop();
                


                wait(10, msec);

                UI();
            }
        }

        // Main function
            int main() {
                Brain.Timer.clear();
                Brain.Screen.print("\nCalibrating inertial sensor...\n");
                //printf("\nCalibrating inertial sensor...\n");
            
                inertialSensor.calibrate();
                while (inertialSensor.isCalibrating()) {
                    wait(10, msec);
                }
            
                double calibrationTime = Brain.Timer.time(seconds);
                Brain.Screen.print("Calibration complete! Time taken: %.2f seconds\n", calibrationTime);
                //printf("Calibration complete! Time taken: %.2f seconds\n", calibrationTime);
            
                vex::task UIUpdateTask(UIUpdate);
            
                // Don't start user control until calibration is done
                usercontrol();
            }

