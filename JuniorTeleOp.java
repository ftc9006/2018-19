package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="JuniorTeleOp", group="Iterative Opmode")
//@Disabled
public class JuniorTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HitlersHardware robot       = new HitlersHardware();
    int sweepGo=0;


    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop()
    {

        double leftPower;
        double rightPower;

        leftPower = gamepad1.left_stick_y;
        rightPower  =  gamepad1.right_stick_y;

        // Drive Mechanics
        if (gamepad1.left_bumper == true)
        {
            robot.leftRearDrive.setPower(1);
            robot.leftFrontDrive.setPower(-1);
            robot.rightRearDrive.setPower(-1);
            robot.rightFrontDrive.setPower(1);
        }
        else if (gamepad1.right_bumper == true)
        {
            robot.leftRearDrive.setPower(-1);
            robot.leftFrontDrive.setPower(1);
            robot.rightRearDrive.setPower(1);
            robot.rightFrontDrive.setPower(-1);
        }
        else if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0)
        {
            robot.leftRearDrive.setPower(rightPower);
            robot.leftFrontDrive.setPower(rightPower);
            robot.rightRearDrive.setPower(leftPower);
            robot.rightFrontDrive.setPower(leftPower);
        }
       else
        {
            robot.leftRearDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
        }

        // Lifter Mechanics
        if (gamepad1.dpad_up == true && robot.touchUp.getState() == true)
            robot.Lifter.setPower(1);
        else if (gamepad1.dpad_down == true && robot.touchDown.getState() == true)
            robot.Lifter.setPower(-1);
        else
            robot.Lifter.setPower(0);

        // Sweeper Mechanics
        if(gamepad1.x)
        {
            if(sweepGo == 0) {
                telemetry.addData("Status", "hit x");
                robot.sweeperLeft.setPower(1);
                robot.sweeperRight.setPower(1);
                sweepGo = 1;
            }
            else if(sweepGo==1)
            {
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                sweepGo=0;
            }
        }
        else if(gamepad1.b)
        {
            if(sweepGo == 0) {
                telemetry.addData("Status", "hit x");
                robot.sweeperLeft.setPower(-1);
                robot.sweeperRight.setPower(-1);
                sweepGo = 1;
            }
            else if(sweepGo==1) {
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                sweepGo = 0;
            }
        }



        // Arm Mechanics
        if(gamepad1.a)
        {
            robot.ArmLeft.setPower(.5);
            robot.ArmRight.setPower(.5);
        }
        else if(gamepad1.y && robot.touchArm.getState() == true)
        {
            robot.ArmLeft.setPower(-.5);
            robot.ArmRight.setPower(-.5);
        }
        else
        {
            robot.ArmLeft.setPower(0);
            robot.ArmRight.setPower(0);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    @Override
    public void stop() {}
}
