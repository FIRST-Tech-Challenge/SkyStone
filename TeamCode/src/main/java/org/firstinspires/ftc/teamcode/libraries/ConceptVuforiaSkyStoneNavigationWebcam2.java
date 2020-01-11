//package org.firstinspires.ftc.teamcode.libraries;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.VUFORIA_KEY;
//
//@Autonomous(name = "USE FOR BLUE SIDE", group = "Concept")
//
//public class ConceptVuforiaSkyStoneNavigationWebcam2 extends LinearOpMode {
//
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final boolean PHONE_IS_PORTRAIT = false;
//
//
////    private static final String VUFORIA_KEY = "ARSzhHP/////AAABmQ3dyIKKfkcipjZh0HtnoDEkjuCn18CTNUWRN7PTFoedxZLS+QZmpkyXpQnQXFpQ5ol//l0ZwTejVrGRQ4i/kQBrrFJ8E0C7ckr4lzf5bLCvi1/E9x8anPwt2D0UToZ3MB5jPx4T6s/EOs575BtxjL7uv5jrCbQDsXebm2PROU4zC/Dj7+AYFkKCqD3YYLbGPGV4YoSgp9Ihoe+ZF/eae0FLG8K/o4eyfZj0B3aXkRvYi3dC5LY+c76aU72bKTrQ2PDYSxDG8xCaY1JyEyfDA6XqjHjYMvh0BBbb8bAQvPgG6/G50+5L+c/a8u6sbYJLbvVtXdMtrG1EA4CglbnsDs7GyyJmH5AusSwIDb9DQnTA";
//
//
//    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
//    // We will define some constants and conversions here
//    private static final float mmPerInch = 25.4f;
//    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
//
//    // Constant for Stone Target
//    private static final float stoneZ = 2.00f * mmPerInch;
//
//    // Constants for the center support targets
//    private static final float bridgeZ = 6.42f * mmPerInch;
//    private static final float bridgeY = 23 * mmPerInch;
//    private static final float bridgeX = 5.18f * mmPerInch;
//    private static final float bridgeRotY = 59;                                 // Units are degrees
//    private static final float bridgeRotZ = 180;
//
//    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField = 36 * mmPerInch;
//
//    String positionSkystone = "";
//    double yPosition = 0;
//    double xPosition = 0;
//    boolean startIdentify = true;
//    float distanceToDepot = 110;    //115
//    float distanceToCenterLine = 5.5f;
//    float forwardDistanceSkystone = 28f;
//    float turningDegree = -50;
//    float foundation = 14;
//
//    // Class Members
//    private OpenGLMatrix lastLocation = null;
//    private VuforiaLocalizer vuforia = null;
//
//
//    WebcamName webcamName = null;
//
//    private boolean targetVisible = false;
//    private float phoneXRotate = 0;
//    private float phoneYRotate = 0;
//    private float phoneZRotate = 0;
//    private AutoLib autoLib;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
//
//        /*
//         * Retrieve the camera we are to use.
//         */
//
//
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        parameters.cameraName = webcamName;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");
//
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(targetsSkyStone);
//
//        stoneTarget.setLocation(OpenGLMatrix
//                .translation(0, 0, stoneZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        blueFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
//
//        blueRearBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
//
//        redFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
//
//        redRearBridge.setLocation(OpenGLMatrix
//                .translation(bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90;
//        }
//
//        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
//        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
//
//        targetsSkyStone.activate();
//        if (startIdentify) {
//            autoLib.calcMove(20, 1f, Constants.Direction.BACKWARD);    //46
//            Thread.sleep(1000);
//            while (!isStopRequested() && startIdentify) {
//
//                targetVisible = false;
//                for (VuforiaTrackable trackable : allTrackables) {
//                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                        telemetry.addData("Visible Target", trackable.getName());
//                        targetVisible = true;
//
//                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                        if (robotLocationTransform != null) {
//                            lastLocation = robotLocationTransform;
//                        }
//                        break;
//                    }
//                }
//
////            String positionSkystone = "";
////            double yPosition = 0;
////            double xPosition = 0;
//                if (targetVisible) {
//                    // express position (translation) of robot in inches.
//                    VectorF translation = lastLocation.getTranslation();
//                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                    yPosition = translation.get(1);
//                    xPosition = translation.get(0);
//                    if (yPosition < 0) {
//                        positionSkystone = "Left";
////                        autoLib.calcMove(1, .7f, Constants.Direction.LEFT);
//                        finalMove(-xPosition, yPosition);
//                        distanceToDepot = distanceToCenterLine + 6;
//                    } else {
//                        positionSkystone = "Center";
//                        //if (xPosition <= -25) {
//                        distanceToDepot = distanceToDepot + 15; //20
//                        forwardDistanceSkystone = forwardDistanceSkystone + 3;
//                        distanceToCenterLine = distanceToCenterLine + 1;
//                        turningDegree = turningDegree - 1f;
//                        foundation = foundation + 4;
//                        turningDegree = turningDegree - 1;
////                        turningDegree = turningDegree - 10f;
//                        sleep(750);
//                        yPosition = translation.get(1);
//                        xPosition = translation.get(0);
//                        finalMove(-xPosition, yPosition);
//                        break;
////                        } else {
////                            telemetry.addData("Final Position Reached", "none");
////                        }
//                    }
//
//                    // express the rotation of the robot in degrees.
//                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                } else {
//                    positionSkystone = "Right";
//                    telemetry.addData("Visible Target", "none");
//
//                    distanceToDepot = distanceToDepot + 10;
//                    distanceToCenterLine = distanceToCenterLine + 1;
//                    forwardDistanceSkystone = forwardDistanceSkystone - 1;
//                    foundation = foundation - 3;
//
//                    autoLib.calcMove(20, .7f, Constants.Direction.RIGHT);
//                    Thread.sleep(1000);
//
//                }
//                telemetry.addData("Skystone Position", positionSkystone);
//                telemetry.update();
//            }
//        }
//
//        targetsSkyStone.deactivate();
//    }
//
//    private void finalMove(double xPosition, double yPosition) throws InterruptedException {
//        telemetry.addData("Final Position Reached", "none");
//        telemetry.addData("X Position ", xPosition);
//        telemetry.addData("Y Position ", yPosition);
//// go near skystone
////        autoLib.moveArmDownScoreServoArmGrab();
//
//        autoLib.calcMove((float) (yPosition / 10) + distanceToCenterLine, .9f, Constants.Direction.RIGHT); //when decreased- moves to the left
//        autoLib.calcMove((float) (-xPosition / 10) + forwardDistanceSkystone, .5f, Constants.Direction.FORWARD);   //when increased-moves back
////        distanceToDepot = distanceToDepot + (float) yPosition + 5;
//        autoLib.calcMove(5f, .7f, Constants.Direction.BACKWARD);
//        Thread.sleep(500);
////        autoLib.armG/rab();
//        Thread.sleep(500);
//        autoLib.calcMove(17f, .8f, Constants.Direction.FORWARD);    //16
//        autoLib.calcTurn((int) turningDegree, .7f); //53
////        if (distanceToDepot > 120) {//195
////            distanceToDepot = 130;//205
////        }
//        autoLib.calcMove(distanceToDepot, 1f, Constants.Direction.BACKWARD);
////        autoLib.moveArmUpSeconds();
//        autoLib.calcTurn(50, .6f);
//        autoLib.calcMove(foundation, .7f, Constants.Direction.BACKWARD);
////        autoLib.scoreServo();
//        autoLib.calcMove(5, .15f, Constants.Direction.BACKWARD);
//        Thread.sleep(300);
////        autoLib.latchServoFoundation();
//        Thread.sleep(1000);
//        autoLib.calcMove(60, 1f, Constants.Direction.FORWARD);
//        autoLib.restServoFoundation();
//        autoLib.calcMove(72, 1f, Constants.Direction.RIGHT);
//        startIdentify = false;
//
//    }
//
//    private void initialize() {
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        autoLib = new AutoLib(this);
//
//        telemetry.addData("Status", "Ready");
//        telemetry.update();
//        waitForStart();
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//    }
//}
package org.firstinspires.ftc.teamcode.libraries;

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


@TeleOp(name = "BlueSide", group = "Concept")

public class ConceptVuforiaSkyStoneNavigationWebcam2 extends LinearOpMode {

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
    float distanceToDepot = 165;

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
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
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
            autoLib.calcMove(15, .9f, Constants.Direction.BACKWARD);    //43
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

                // Provide feedback as to where the robot is located (if we know).
//            String positionSkystone = "";
//            double yPosition = 0;
//            double xPosition = 0;
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    yPosition = translation.get(1);
                    xPosition = translation.get(0);
                    if (yPosition < 0) {
                        positionSkystone = "Right";
//                        autoLib.calcMove(3, .8f, Constants.Direction.RIGHT);
                        distanceToDepot = distanceToDepot + 10;
                        //  sleep(3000);
                    } else {
                        positionSkystone = "Center";
                        //if (xPosition <= -25) {
                        sleep(1000);
                        yPosition = translation.get(1);
                        xPosition = translation.get(0);
//                        finalMove(-xPosition, yPosition);
                      //  break;
//                        } else {
//                            telemetry.addData("Final Position Reached", "none");
//                        }
                    }

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    positionSkystone = "Left";
                    telemetry.addData("Visible Target", "none");

//                    autoLib.calcMove(5, .8f, Constants.Direction.LEFT);

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
// go near skystone
        autoLib.calcMove((float) (-xPosition / 10) + 20, .8f, Constants.Direction.FORWARD);   //changed
        autoLib.calcMove((float) (yPosition / 10) + 10, .8f, Constants.Direction.LEFT);
        distanceToDepot = distanceToDepot + (float) yPosition;

        //autoLib.calcMove(20, .5f, Constants.Direction.BACKWARD);
        Thread.sleep(500);
        autoLib.scoreServoArm();
        autoLib.grabServo();
        Thread.sleep(500);
        // autoLib.calcMove(10, .8f, Constants.Direction.LEFT);
        autoLib.calcMove(9, .8f, Constants.Direction.FORWARD);
        autoLib.calcMove(distanceToDepot, .8f, Constants.Direction.RIGHT);
        autoLib.calcMove(38, .7f, Constants.Direction.BACKWARD);
        autoLib.latchServoFoundation();
        Thread.sleep(1000);
        autoLib.calcMove(70, 1f, Constants.Direction.FORWARD);
        autoLib.restServoFoundation();
        autoLib.calcMove(distanceToDepot - 100, 1f, Constants.Direction.LEFT);

        //Stop intake
//        autoLib.calcMove(10, .5f, Constants.Direction.LEFT);
//        autoLib.scoreServoArm();
//        autoLib.grabServo();
        // come back
//        autoLib.calcMove(70, .8f, Constants.Direction.FORWARD);
        //go to foundation
//        autoLib.calcMove(225, .8f, Constants.Direction.RIGHT);
//        autoLib.calcMove(200, .8f, Constants.Direction.BACKWARD);
        // Drop
//        autoLib.calcMove(115, .8f, Constants.Direction.RIGHT);
//        autoLib.calcMove(5,.8f, Constants.Direction.FORWARD);
//        autoLib.latchServoFoundation();
//        autoLib.calcMove(210, .8f, Constants.Direction.FORWARD);
//        autoLib.calcMove(200, .8f, Constants.Direction.LEFT);
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

