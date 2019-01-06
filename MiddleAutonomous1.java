/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Middle", group="Pushbot")
@Disabled
public class MiddleAutonomous1 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    HardwarePushbotChain robot = new HardwarePushbotChain();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;
        int no =1;

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
        }
        while (opModeIsActive())

            // Sense Gold Cude
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }

            if(no == 1) {
                // Move Away From Wall
                robot.leftFDrive.setPower(1);
                robot.leftRDrive.setPower(1);
                robot.rightFDrive.setPower(1);
                robot.rightRDrive.setPower(1);
                sleep(250);

                // Stop
                robot.leftFDrive.setPower(0);
                robot.leftRDrive.setPower(0);
                robot.rightFDrive.setPower(0);
                robot.rightRDrive.setPower(0);
                sleep(1000);

                // Rotate
                robot.leftFDrive.setPower(1);
                robot.leftRDrive.setPower(1);
                robot.rightFDrive.setPower(-1);
                robot.rightRDrive.setPower(-1);
                sleep(500);

                // Stop
                robot.leftFDrive.setPower(0);
                robot.leftRDrive.setPower(0);
                robot.rightFDrive.setPower(0);
                robot.rightRDrive.setPower(0);
                sleep(1000);

                // Drive to Cube and Do The Thing
                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) // Left Side
                {
                    // Line Up
                    robot.leftFDrive.setPower(-1);
                    robot.leftRDrive.setPower(-1);
                    robot.rightFDrive.setPower(-1);
                    robot.rightRDrive.setPower(-1);
                    sleep(250);

                    // Stop
                    robot.leftFDrive.setPower(0);
                    robot.leftRDrive.setPower(0);
                    robot.rightFDrive.setPower(0);
                    robot.rightRDrive.setPower(0);
                    sleep(1000);

                    // Rotate
                    robot.leftFDrive.setPower(1);
                    robot.leftRDrive.setPower(1);
                    robot.rightFDrive.setPower(-1);
                    robot.rightRDrive.setPower(-1);
                    sleep(500);
                    no = 0;
                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) // Right Side
                {
                    // Line Up
                    robot.leftFDrive.setPower(1);
                    robot.leftRDrive.setPower(1);
                    robot.rightFDrive.setPower(1);
                    robot.rightRDrive.setPower(1);
                    sleep(250);

                    // Stop
                    robot.leftFDrive.setPower(0);
                    robot.leftRDrive.setPower(0);
                    robot.rightFDrive.setPower(0);
                    robot.rightRDrive.setPower(0);
                    sleep(1000);

                    // Rotate
                    robot.leftFDrive.setPower(1);
                    robot.leftRDrive.setPower(1);
                    robot.rightFDrive.setPower(-1);
                    robot.rightRDrive.setPower(-1);
                    sleep(500);

                    // Stop
                    robot.leftFDrive.setPower(0);
                    robot.leftRDrive.setPower(0);
                    robot.rightFDrive.setPower(0);
                    robot.rightRDrive.setPower(0);
                    sleep(1000);
                    no = 0;
                } else // Center
                {
                    // Rotate
                    robot.leftFDrive.setPower(1);
                    robot.leftRDrive.setPower(1);
                    robot.rightFDrive.setPower(-1);
                    robot.rightRDrive.setPower(-1);
                    sleep(500);

                    // Stop
                    robot.leftFDrive.setPower(0);
                    robot.leftRDrive.setPower(0);
                    robot.rightFDrive.setPower(0);
                    robot.rightRDrive.setPower(0);
                    sleep(1000);
                    no = 0;
                }

                // Drive to Hit
                robot.leftFDrive.setPower(-1);
                robot.leftRDrive.setPower(-1);
                robot.rightFDrive.setPower(-1);
                robot.rightRDrive.setPower(-1);
                sleep(250);

                // Stop
                robot.leftFDrive.setPower(0);
                robot.leftRDrive.setPower(0);
                robot.rightFDrive.setPower(0);
                robot.rightRDrive.setPower(0);
                sleep(1000);

                // Drive Away
                robot.leftFDrive.setPower(1);
                robot.leftRDrive.setPower(1);
                robot.rightFDrive.setPower(1);
                robot.rightRDrive.setPower(1);
                sleep(250);

                // Stop
                robot.leftFDrive.setPower(0);
                robot.leftRDrive.setPower(0);
                robot.rightFDrive.setPower(0);
                robot.rightRDrive.setPower(0);
                sleep(1000);

            }

        }

        // Stop all motors
        robot.leftFDrive.setPower(0);
        robot.leftRDrive.setPower(0);
        robot.rightFDrive.setPower(0);
        robot.rightRDrive.setPower(0);
    }
}

