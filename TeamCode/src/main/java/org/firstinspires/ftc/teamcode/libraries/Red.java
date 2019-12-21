package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "PICK ME FOR RED!!!", group = "Concept")

public class Red extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;


//    private static final String VUFORIA_KEY = "ARSzhHP/////AAABmQ3dyIKKfkcipjZh0HtnoDEkjuCn18CTNUWRN7PTFoedxZLS+QZmpkyXpQnQXFpQ5ol//l0ZwTejVrGRQ4i/kQBrrFJ8E0C7ckr4lzf5bLCvi1/E9x8anPwt2D0UToZ3MB5jPx4T6s/EOs575BtxjL7uv5jrCbQDsXebm2PROU4zC/Dj7+AYFkKCqD3YYLbGPGV4YoSgp9Ihoe+ZF/eae0FLG8K/o4eyfZj0B3aXkRvYi3dC5LY+c76aU72bKTrQ2PDYSxDG8xCaY1JyEyfDA6XqjHjYMvh0BBbb8bAQvPgG6/G50+5L+c/a8u6sbYJLbvVtXdMtrG1EA4CglbnsDs7GyyJmH5AusSwIDb9DQnTA";


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
    float distanceToDepot = 110;    //115
    float distanceToCenterLine = 5.5f;
    float forwardDistanceSkystone = 28f;
    float turningDegree = -50;
    float foundation = 14;

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

//        autoLib.moveArmDownScoreServoArmGrab();
//        autoLib.calcMove(10, .5f, Constants.Direction.RIGHT);
//        autoLib.calcMove(40, .7f, Constants.Direction.BACKWARD);
//        autoLib.calcMove(4, .3f, Constants.Direction.BACKWARD);
//        Thread.sleep(500);
//        autoLib.grabServo();
//        Thread.sleep(500);
//        autoLib.calcMove(22f, .8f, Constants.Direction.FORWARD);    //16
//        autoLib.calcTurn(52, .4f); //53
//        autoLib.calcMove(115, 1f, Constants.Direction.BACKWARD);
//        autoLib.moveArmUpSeconds();
//        autoLib.calcTurn(-50, .6f);
//        autoLib.calcMove(18.5f, .7f, Constants.Direction.BACKWARD);
//        Thread.sleep(1000);
//        autoLib.scoreServo();
//        Thread.sleep(500);
//        autoLib.calcMove(9, .15f, Constants.Direction.BACKWARD);
//        Thread.sleep(300);
//        autoLib.latchServoFoundation();
//        Thread.sleep(1000);
//        autoLib.calcMove(80, 1f, Constants.Direction.FORWARD);
////        autoLib.calcTurn(88,.6f);
//        autoLib.restServoFoundation();
//        autoLib.calcMove(65, 1f, Constants.Direction.FORWARD);
        autoLib.calcMove(45,.45f, Constants.Direction.BACKWARD);


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
