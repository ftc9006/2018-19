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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *this code is trash
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="JuniorTeleOp", group="Iterative Opmode")
@Disabled
public class JuniorTeleOpGeorge extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftRearDrive = null;
    //private DcMotor rightRearDrive = null;
    //private DcMotor leftFrontDrive = null;
    //private DcMotor rightFrontDrive = null;
    HitlersHardware robot       = new HitlersHardware();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

    //    leftRearDrive  = hardwareMap.get(DcMotor.class, "left_Rear_drive");
    //    rightRearDrive = hardwareMap.get(DcMotor.class, "right_Rear_drive");
    //    leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_Front_drive");
    //    rightFrontDrive = hardwareMap.get(DcMotor.class, "right_Front_drive");

    //    leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
    //    rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
    //    leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    //    rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double leftPower;
        double leftleftPower;
        double rightPower;
        double rightrightPower;

        leftPower = -gamepad1.left_stick_y;
        rightPower  =  gamepad1.right_stick_y;
        leftleftPower = -gamepad1.left_stick_x;
        rightrightPower = gamepad1.left_stick_x;

        robot.leftRearDrive.setPower(leftPower);
        robot.rightRearDrive.setPower(rightPower);
        robot.leftFrontDrive.setPower(leftPower);
        robot.rightFrontDrive.setPower(rightPower);

        if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0)
        {
            robot.leftRearDrive.setPower(leftleftPower);
            robot.rightRearDrive.setPower(-rightrightPower);
            robot.leftFrontDrive.setPower(-leftleftPower);
            robot.rightFrontDrive.setPower(rightrightPower);
        }
        else if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0)
        {
            robot.leftRearDrive.setPower(-leftleftPower);
            robot.rightRearDrive.setPower(rightrightPower);
            robot.leftFrontDrive.setPower(leftleftPower);
            robot.rightFrontDrive.setPower(-rightrightPower);
        }




 /*
        double sideRight, sideLeft;
        boolean straffeL,straffeR;


        sideRight= gamepad1.left_stick_x;
        sideLeft= gamepad1.right_stick_x;
        straffeL = gamepad1.right_bumper;
        straffeR = gamepad1.left_bumper;





        if(sideRight!=0&&sideLeft<0) {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft * -1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideRight!=0&&sideLeft>0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideLeft!=0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(straffeL==true)
        {
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.2);
            robot.rightRearDrive.setPower(.2);
        }
        else if(straffeR==true)
        {
            robot.leftFrontDrive.setPower(-.2);
            robot.rightFrontDrive.setPower(.2);
            robot.leftRearDrive.setPower(.2);
            robot.rightRearDrive.setPower(-.2);
        }
        else
        {
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
        }
*/

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    @Override
    public void stop() {
    }

}
