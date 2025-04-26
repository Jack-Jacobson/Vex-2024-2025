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
        #include <functional>
        #include <vector>

        using namespace vex;

        competition Competition;

        vex::brain Brain;
        vex::controller Controller;

        // Motor and sensor declarations
        motor frontLeftDrive = motor(PORT3, ratio6_1, true);
        motor frontRightDrive = motor(PORT1, ratio6_1, true);
        motor middleLeftDrive = motor(PORT6, ratio6_1, true);
        motor middleRightDrive = motor(PORT5, ratio6_1, false);
        motor backLeftDrive = motor(PORT2, ratio6_1, false);
        motor backRightDrive = motor(PORT4, ratio6_1, false);
       motor intake = motor(PORT20, ratio6_1, true);
      //  motor LB = motor(PORT2, ratio6_1, false);

        digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);
        digital_out doinker = digital_out(Brain.ThreeWirePort.D);

        encoder LBRotation = encoder(Brain.ThreeWirePort.G);

        inertial inertialSensor = inertial(PORT12);

        motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);
        motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

        // Variable declarations
        int leftVel = 0, rightVel = 0, fwdSpeed; // Velocity of each side of the robot
        int touchX, touchY; // Variables for screen touch coordinates
        int currentAuton = 2; // Selected autonomous routine
        bool mogoClamped = false; // State of the mogo clamp
        bool fastTurn = false; // Toggle for fast turning
        bool intertialCalibrated = false;
        double kP = 0.1, integral = 0.0, kI = 0.00, kD = 0.008, previousError = 0, LeftPIDTurningOutput = 0, RightPIDTurningOutput = 0; // PID constants
        int counter = 0;
        int counter2 = 0;
        double targetHeading, threshold = 2, tkP = 0.12, tkI = 0.005, tkD = 0.06; // PID variables
        double leftPIDOutput, rightPIDOutput; // PID output variables
        double power = 0.0; // Power variable for motors
        int currentHeading = 0; // Current heading from the inertial sensor

        double totalDegrees = 0.0, MMTravelled = 0.0; // Total degrees and millimeters traveled
        double prevLeft = 0.0;
        double prevRight = 0.0;

        //double fwdSpeed = 60; // Forward velocity


        struct Task {
            double dist;
            double fwdSpeed;
            double targetHeading;
            int additionalEvent;
        };
        



        std::string screen = "main"; // Current screen displayed on the brain

        int degreeTrackerTask() {
            while (true) {
                  
                double left = leftDrive.position(degrees);
                double right = rightDrive.position(degrees);

                double deltaLeft = left - prevLeft;
                double deltaRight = right - prevRight;

                if ((deltaLeft > 0 && deltaRight > 0) || (deltaLeft < 0 && deltaRight < 0)) {
                    double avgDeltaDeg = (deltaLeft + deltaRight) / 2.0;
                    totalDegrees += abs(avgDeltaDeg);
                }

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
               // Brain.Screen.print(intake.temperature(celsius));
                Brain.Screen.print(",    Lady Brown: ");
               // Brain.Screen.print(LB.temperature(celsius));
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
                Brain.Screen.print(LBRotation.position(degrees));
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
                currentHeading = inertialSensor.rotation(degrees);
                double error = targetHeading - currentHeading;

                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                // PID calculations
                double proportionOutput = tkP * error;

                if (fabs(error) < 15) integral += error;
                else integral = 0;
                double integralOutput = integral * tkI;
                
                double derivative = error - previousError;
                double derivativeOutput = tkD * derivative;

                power = proportionOutput + integralOutput + derivativeOutput;

                if (power > 6) power = 6;
                if (power < -6) power = -6;
                

                //printf("Power: %.2f | Heading: %.2f | Error: %.2f | Previous Error: %.2f | P Gain: %.2f | I Gain: %.2f | D Gain: %.2f \n", power, currentHeading, error, previousError, proportionOutput, integralOutput, derivativeOutput);    

                backLeftDrive.setReversed(true);
                frontRightDrive.setReversed(false);
                //leftDrive.spin(forward,fwdSpeed + Controller.Axis3.position()*0.12 + Controller.Axis1.position()*0.06  , volt);
                //rightDrive.spin(forward, fwdSpeed + Controller.Axis3.position()*0.12 - Controller.Axis1.position()*0.09, volt);

                previousError = error;

                

                vex::this_thread::sleep_for(10);
            }
                
        }

        vex::task turnPIDTask(turnToHeading);

        int UIUpdate(void) {
            while (true) {
                //printf("Current heading: %.2f | Target heading: %.2f | Left PID Output: %.2f | Right PID Output: %.2f\n", inertialSensor.heading(degrees), targetHeading, leftPIDOutput, rightPIDOutput);
                //printf("xPosition: %.2f | yPosition: %.2f | Left Drive Position: %.2f | Right Drive Position: %.2f\n", posX, posY, leftDrive.position(degrees), rightDrive.position(degrees));
                //printf("fwdSpeed: %.2f  |  PIDOutput: %.2f  |  currentHeading: %.2f |  distanceTravelled: %.2f\n", fwdSpeed, power, inertialSensor.heading(degrees), totalDegrees);
                //printf("Distance Travelled: %.2f\n", totalDegrees);

                vex::this_thread::sleep_for(100);
            }
        }

        int checkEvents(void) {
            std::vector<Task> tasks;
            int currentTask = 0;

            // Define tasks with distance, speed, target heading, and additional events
           /* tasks.push_back({0, 8, 0, 0});       // Example task 1
            tasks.push_back({1600, -8, 0, 2});   // Example task 2
            tasks.push_back({1900, 0, 180, 2});  // Example task 3
            tasks.push_back({0, 0, 180, 2});     // Example task 4 */

            while (true) {
                // Calculate distance traveled in millimeters
                MMTravelled = (totalDegrees / 360) * M_PI * 82.55;

                // Check if the current task's distance has been reached
                if (MMTravelled >= tasks[currentTask].dist) {
                    fwdSpeed = tasks[currentTask].fwdSpeed;
                    targetHeading = tasks[currentTask].targetHeading;

                    // Wait for the turn to complete within the 2-degree tolerance
                    while (fabs(inertialSensor.rotation(degrees) - targetHeading) > 2) {
                        // Continuously check the heading
                        vex::this_thread::sleep_for(10);
                    }

                    // Trigger the additional event for the current task
                    if (tasks[currentTask].additionalEvent == 1) {
                        mogoClamp.set(!mogoClamp);
                    } else if (tasks[currentTask].additionalEvent == 2) {
                        doinker.set(!doinker);
                    } else if (tasks[currentTask].additionalEvent == 3) {
                        if (intake.velocity(percent) == 0) {
                            intake.spin(forward, 100, percent);
                        } else {
                            intake.stop();
                        }
                    } else if (tasks[currentTask].additionalEvent == 4) {
                        if (intake.velocity(percent) == 0) {
                            intake.spin(reverse, 100, percent);
                        } else {
                            intake.stop();
                        }
                    }

                    // Print debug information
                    printf("Current Task: %d\n", currentTask);
                    printf("fwdSpeed: %.2f\n", fwdSpeed);
                    printf("Target Heading: %.2f\n", targetHeading);
                    printf("Distance travelled: %.2f\n", MMTravelled);

                    // Move to the next task
                    currentTask++;
                }

                // Check if all tasks are completed
                if (currentTask >= tasks.size()) {
                    printf("All tasks completed.\n");
                    return 0;
                }

                // Sleep to prevent excessive CPU usage
                vex::this_thread::sleep_for(5);
            }
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
            while (true) {
                // Drive control: Left stick controls forward/backward, right stick controls turning
                int fwd = Controller.Axis3.position()*0.12;
                int turn = Controller.Axis1.position()*0.12;

                // Set motor velocities directly based on controller input
                leftDrive.spin(forward, fwd + turn, volt);
                rightDrive.spin(forward,  fwd - turn, volt);

                // Intake control
                if (Controller.ButtonR2.pressing()) {
                    intake.spin(forward, 12, volt);
                } else if (Controller.ButtonR1.pressing()) {
                    intake.spin(reverse, 12, volt);
                } else {
                    intake.stop();
                }

                // Mogo clamp control
                if (Controller.ButtonL2.pressing()) {
                    mogoClamp.set(!mogoClamped);
                    mogoClamped = !mogoClamped;
                }

                // Wait to prevent excessive CPU usage
                wait(20, msec);
            }
        }

        // Main function
            int main() {
                Brain.Timer.clear();
                Brain.Screen.print("\nCalibrating inertial sensor...\n");
                printf("\nCalibrating inertial sensor...\n");
            
                inertialSensor.calibrate();
                while (inertialSensor.isCalibrating()) {
                    wait(30, msec);
                }
            
                double calibrationTime = Brain.Timer.time(seconds);
                Brain.Screen.print("Calibration complete! Time taken: %.2f seconds\n", calibrationTime);
                printf("Calibration complete! Time taken: %.2f seconds\n", calibrationTime);
            
                vex::task UIUpdateTask(UIUpdate);
                vex::task checkEventsTask(checkEvents);

                // Don't start user control until calibration is done
                usercontrol();

            }

