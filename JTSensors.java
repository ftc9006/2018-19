
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;



@TeleOp(name="JRSensor", group="Iterative Opmode")
@Disabled
public class JTSensors extends OpMode
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    Orientation angles;

    OpenGLMatrix lastLocation = null;
    static float z=0,startZ=0,checkZ=0;
    static float c=0,startC=0,checkC=0;
    int goldMineralX = -1;
    int silverMineral1X = -1;
    int silverMineral2X = -1;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HitlersHardware robot       = new HitlersHardware();
    int sweepGo=0;

    private static final String VUFORIA_KEY = "AQccD+7/////AAAAGZ+qCVwS3U1qhMXGyIILYuuIvQNwDde781BNyGbcw4QWfN1VlGOdTUaQDJNDnVDmPMVVoYtNxOyAhE6u6X7fFdwAP7EYo5HEXo6VbdJKs5f87V+FultdnK29+6hlnHexuPoV6J5NSkbBCCb/K0LYUKLUiAgaxrxi/cbt3O+k06CXjx0SZv/OLKkmwCQBou2oNm6rNmOTjlb82J9JNWKQtDh6No5mHdJ+QGqdqitGK1/eYxZUrnuwCHdXRDgQ7pFUE6CrI6Hi8qrKWLpdNatPfMmGwSFnNlS7O3E50KOiFL7Z46qwn41ROWWV7k9XOEaT7EbBN6UyewgakNmojMytsasBRSTcgc0W/OO2AXvu1UX0";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
     * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters1.vuforiaLicenseKey = VUFORIA_KEY ;
    parameters1.cameraDirection   = CAMERA_CHOICE;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters1);

    // Load the data sets that for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
    VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
    blueRover.setName("Blue-Rover");
    VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
    redFootprint.setName("Red-Footprint");
    VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
    frontCraters.setName("Front-Craters");
    VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
    backSpace.setName("Back-Space");

     For convenience, gather together all the trackable objects in one easily-iterable collection
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsRoverRuckus);

    /**
     * In order for localization to work, we need to tell the system where each target is on the field, and
     * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
     * Transformation matrices are a central, important concept in the math here involved in localization.
     * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     * for detailed information. Commonly, you'll encounter transformation matrices as instances
     * of the {@link OpenGLMatrix} class.
     *
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     *     - The X axis runs from your left to the right. (positive from the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     *
     * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
     *
     * Before being transformed, each target image is conceptually located at the origin of the field's
     *  coordinate system (the center of the field), facing up.
     */

    /**
     * To place the BlueRover target in the middle of the blue perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Then, we translate it along the Y axis to the blue perimeter wall.
     */
    OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
            .translation(0, mmFTCFieldWidth, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
    blueRover.setLocation(blueRoverLocationOnField);

    /**
     * To place the RedFootprint target in the middle of the red perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative Y axis to the red perimeter wall.
     */
    OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
            .translation(0, -mmFTCFieldWidth, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
    redFootprint.setLocation(redFootprintLocationOnField);

    /**
     * To place the FrontCraters target in the middle of the front perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative X axis to the front perimeter wall.
     */
    OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
    frontCraters.setLocation(frontCratersLocationOnField);

    /**
     * To place the BackSpace target in the middle of the back perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the X axis to the back perimeter wall.
     */
    OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
    backSpace.setLocation(backSpaceLocationOnField);

    /**
     * Create a transformation matrix describing where the phone is on the robot.
     *
     * The coordinate frame for the robot looks the same as the field.
     * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
     * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
     *
     * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
     * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
     * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
     *
     * If using the rear (High Res) camera:
     * We need to rotate the camera around it's long axis to bring the rear camera forward.
     * This requires a negative 90 degree rotation on the Y axis
     *
     * If using the Front (Low Res) camera
     * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
     * This requires a Positive 90 degree rotation on the Y axis
     *
     * Next, translate the camera lens to where it is on the robot.
     * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
     */

    final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                    CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

    /**  Let all the trackable listeners know where the phone is.  */
    for (VuforiaTrackable trackable : allTrackables)
    {
        ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }


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


       if (gamepad1.dpad_left)
       {

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


       }
        // Arm Mechanics
        if(gamepad1.dpad_right) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            startZ = AngleUnit.DEGREES.normalize(angles.firstAngle);


        }
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        z = AngleUnit.DEGREES.normalize(angles.firstAngle);
        telemetry.addData("startZ", startZ);
        telemetry.addData("Z", z);

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
