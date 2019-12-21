package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Red Autonomous 29 Points", group = "Concept")

public class RedAutonomous29Points extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;


    private static final String VUFORIA_KEY = "ARSzhHP/////AAABmQ3dyIKKfkcipjZh0HtnoDEkjuCn18CTNUWRN7PTFoedxZLS+QZmpkyXpQnQXFpQ5ol//l0ZwTejVrGRQ4i/kQBrrFJ8E0C7ckr4lzf5bLCvi1/E9x8anPwt2D0UToZ3MB5jPx4T6s/EOs575BtxjL7uv5jrCbQDsXebm2PROU4zC/Dj7+AYFkKCqD3YYLbGPGV4YoSgp9Ihoe+ZF/eae0FLG8K/o4eyfZj0B3aXkRvYi3dC5LY+c76aU72bKTrQ2PDYSxDG8xCaY1JyEyfDA6XqjHjYMvh0BBbb8bAQvPgG6/G50+5L+c/a8u6sbYJLbvVtXdMtrG1EA4CglbnsDs7GyyJmH5AusSwIDb9DQnTA";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    String positionSkystone = "";
    double yPosition = 0;
    double xPosition = 0;
    boolean startIdentify = true;
    float distanceToDepot = 170;    //165
    float distanceToCenterLine = 4.5f;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;


    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private AutoLib autoLib;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // Let all the trackable listeners know where the phone is
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
        if (startIdentify) {
            autoLib.calcMove(46, .7f, Constants.Direction.BACKWARD);
            while (!isStopRequested() && startIdentify) {

                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    yPosition = translation.get(1);
                    xPosition = translation.get(0);
                    if (yPosition >0 && yPosition <=10 ) {
                        positionSkystone = "Right"; //right
                        autoLib.calcMove(3, .5f, Constants.Direction.RIGHT);

                    } else {
                        positionSkystone = "Center";
//                        sleep(1000);
                        yPosition = translation.get(1);
                        xPosition = translation.get(0);
                        finalMove(-xPosition, yPosition);
                        break;

                    }

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    positionSkystone = "Left";  //left
                    telemetry.addData("Visible Target", "none");

                    distanceToDepot = distanceToDepot + 13.5f;

                    autoLib.calcMove(10, .5f, Constants.Direction.LEFT);

                }
                telemetry.addData("Skystone Position", positionSkystone);
                telemetry.update();
            }
        }


        // Disable Tracking when we are done
        targetsSkyStone.deactivate();
    }

    private void finalMove(double xPosition, double yPosition) throws InterruptedException {
        telemetry.addData("Final Position Reached", "none");
        telemetry.addData("X Position ", xPosition);
        telemetry.addData("Y Position ", yPosition);
        telemetry.update();
        //lower arm
        Thread.sleep(500);
        autoLib.moveArmDownScoreServoArmGrab();
// go near skystone
//        autoLib.calcMove((float) (-xPosition / 10) + 14.75f, .9f, Constants.Direction.FORWARD);   //when increased-moves back
//        autoLib.calcMove((float) (yPosition / 10) + 12, .9f, Constants.Direction.RIGHT); //when decreased- moves to the left
//        autoLib.calcMove(3, .3f, Constants.Direction.BACKWARD);
//        autoLib.armGrab();
//        Thread.sleep(250);
//        autoLib.calcMove(25, .9f, Constants.Direction.FORWARD);
//        autoLib.moveArmUpSeconds1();
//        if(distanceToDepot >195)
//        {
//            distanceToDepot=205;
//        }
//        autoLib.calcMove(distanceToDepot, .8f, Constants.Direction.RIGHT);
//        autoLib.moveArmUpSeconds();
//        autoLib.calcMove(38, .9f, Constants.Direction.BACKWARD);
//        autoLib.scoreServo();
//        Thread.sleep(250);
//        autoLib.calcMove(4,.6f, Constants.Direction.BACKWARD);
//        autoLib.latchServoFoundation();
//        Thread.sleep(1000);
//        autoLib.calcMove(80, .9f, Constants.Direction.FORWARD);
//        autoLib.restServoFoundation();
//        autoLib.calcMove(120, 1f, Constants.Direction.LEFT);
//        startIdentify = false;
        autoLib.calcMove((float) (yPosition / 10) + distanceToCenterLine, .9f, Constants.Direction.LEFT); //when decreased- moves to the left
        autoLib.calcMove((float) (-xPosition / 10) + 28f, .6f, Constants.Direction.FORWARD);   //when increased-moves back
//        distanceToDepot = distanceToDepot + (float) yPosition + 5;
        autoLib.calcMove(3.5f, .7f, Constants.Direction.BACKWARD);
        Thread.sleep(500);
        autoLib.armGrab();
        Thread.sleep(500);
        autoLib.calcMove(17f, .8f, Constants.Direction.FORWARD);    //16
        autoLib.calcTurn(55, .7f); //53
        if (distanceToDepot > 120) {//195
            distanceToDepot = 130;//205
        }
        autoLib.calcMove(distanceToDepot, 1f, Constants.Direction.BACKWARD);
        autoLib.moveArmUpSeconds();
        autoLib.calcTurn(50, .6f);
        autoLib.calcMove(10, .7f, Constants.Direction.BACKWARD);
        autoLib.scoreServo();
        autoLib.calcMove(5, .15f, Constants.Direction.BACKWARD);
        Thread.sleep(300);
        autoLib.latchServoFoundation();
        Thread.sleep(1000);
        autoLib.calcMove(60, 1f, Constants.Direction.FORWARD);
        autoLib.restServoFoundation();
        autoLib.calcMove(77, 1f, Constants.Direction.LEFT);
        startIdentify = false;


    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib(this);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
