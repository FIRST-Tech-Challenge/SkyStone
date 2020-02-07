//Graeme Was here

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

//import static org.firstinspires.ftc.teamcode.HardwareK9bot.COLOURSTICK_HOME;

/**
 * Created by student on 11/24/2016.
 */
//comment

public abstract class Autonomous extends LinearOpMode {

    protected HardwareK9bot robot = new HardwareK9bot();

    private int delay;
    private static int TICKS_PER_REVOLUTION = 1440;
    private static double GEAR_RATIO = 0.48; //used to be 0.5
    private static double WHEEL_BASE = 15.3; //used to be 8.0
    private static double TICKS_PER_INCH = TICKS_PER_REVOLUTION * GEAR_RATIO / (Math.PI * 4);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "ATz1+9P/////AAABmeqS5/62ZUGpp5bTjFOlpkUQ/xkdYMvOFM8cjbv7n7uq3sYzUf93tbck4Wwz4tLtprq66GBhDQn1s06gkPiK4MJqUHZsdytuNcFacZO/2S66hK08CjwewQE8Wqs1T8I3wIEQENcMkWha0xwyR/2JfDGwQEGPnO56etL1eXzhScwqGARW1kOAS/zSzg4aWBUITk5FvDZG3lMxpZWIFEOmCIO92DR70BAc8QJz+51mzXvdFSb1kcwkvwcNWQ78ZRfnS41hq84A6Ps84PJRij48wy1oonI2tEXx/RHwoWOBcBFev7VNBDLWCo5VFQ3TtBJeHne5STFubET+3Eg1YWcuFhcAIc2zmVrh/W36NY6a4wkl";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean initFlag = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        runPath();
    }

    abstract protected void runPath();

//    protected void move(int distance, double power) {
//
//        //power = -power;
//
//        //distance = distance * (2 / 3);
//
//        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        //gives power to motors
//        robot.leftMotor.setPower(power);
//        robot.rightMotor.setPower(power);
//
//        idle();
//
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //keep going until distance is met
//        while( (Math.abs(robot.rightMotor.getCurrentPosition()) < distance * TICKS_PER_INCH) && opModeIsActive() ){
//
//            telemetry.addData("Position: ", robot.rightMotor.getCurrentPosition());
//            telemetry.addData("Goal", distance * TICKS_PER_INCH);
//            telemetry.update();
//            idle();
//        }
//        //turn the motors off
//        robot.leftMotor.setPower(0);
//        robot.rightMotor.setPower(0);
//
//        sleep(100); //give the robot the time to lose its momentum
//    }
protected void move(double distance, double power, int direction) {
    robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // old distance // distance = distance * 0.611111111; //used to be 0.55555 (5/9)
    distance = distance * 1.7094017094; //(2.5/2)*0.83333333333;
           //sed to be 0.55555 (5/9)


        //gives power to motors
        robot.frontLeftMotor.setPower((power*2) - ((power*2)*2));
        robot.frontRightMotor.setPower((power*2) - ((power*2)*(direction*2)));
        robot.backLeftMotor.setPower(((power*2) - ((power*2)*2)) - (((power*2) - ((power*2)*2))*(direction*2)));
        robot.backRightMotor.setPower(power*2);

        idle();

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //keep going until distance is met
        while( (Math.abs(robot.backRightMotor.getCurrentPosition()) < distance * TICKS_PER_INCH) && opModeIsActive() ){

            telemetry.addData("position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("%", Math.abs(robot.frontRightMotor.getCurrentPosition())/distance * TICKS_PER_INCH*100);
            telemetry.addData("Ticks Per Inch", distance * TICKS_PER_INCH);
            telemetry.addData("Current Position", Math.abs(robot.backRightMotor.getCurrentPosition()));

            telemetry.update();
            idle();


        }

        //turn the motors off
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        sleep(100); //give the robot the time to lose its momentum




}

   /* protected void move(double distance, double power) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distance = distance * 0.611111111; //used to be 0.55555 (5/9)

        //gives power to motors
        robot.leftMotor.setPower(2*power);
        robot.rightMotor.setPower(2*power);

        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //keep going until distance is met
        while( (Math.abs(robot.leftMotor.getCurrentPosition()) < distance * TICKS_PER_INCH) && opModeIsActive() ){

            telemetry.addData("position", robot.leftMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //turn the motors off
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        sleep(100); //give the robot the time to lose its momentum
    } */

    protected void pivot(int angle, double power) {

        double d = Math.toRadians(angle) * WHEEL_BASE/2;
        d *= .5;
        d += 0.003 * (double)angle;

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //gives power to motors
        power = power * 2;
        robot.frontLeftMotor.setPower(-Math.signum(angle) * power);
        robot.frontRightMotor.setPower(Math.signum(angle) * power);

        idle();

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //keep going until distance is met
        while( (Math.abs(robot.backRightMotor.getCurrentPosition()) < Math.abs(d) * TICKS_PER_INCH) && opModeIsActive() ){
            telemetry.addData("target",d*TICKS_PER_INCH);
            telemetry.addData("current", robot.backRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //turn the motors off
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        sleep(100); //give the robot the time to lose its momentum

    }

    void markerKnock (/*boolean isReversed*/){

        //robot.markerStick.setPosition(0);

        //if (isReversed == false){

            boolean flag = false;

            //robot.markerStick.setDirection(Servo.Direction.FORWARD);
            //robot.markerStick.scaleRange(0,1);
            robot.markerStick.setPosition(1);
//            while (flag){
            //sleep(2000);
//
            try{Thread.sleep(2000);}
            catch(Exception e){
                telemetry.addData("wait", "failed");
                //sleep(5000);
            }

            //robot.markerStick.setPosition(1);
        //}
        //else{
            robot.markerStick.setPosition(0);
        //}

    }

    void detach(){

       // robot.linSlideLeft.setPower(0.5);
       // robot.linSlideRight.setPower(0.5);

//        robot.linSlideLeft.setTargetPosition(5);
//        robot.linSlideRight.setTargetPosition(5);

       /* while (robot.linSlideLeft.getCurrentPosition() < 5340 && opModeIsActive()){
        }

        robot.linSlideLeft.setPower(0);
        robot.linSlideRight.setPower(0);
*/
        sleep(1000);

        //robot.linSlideLeft.setPower(-0.5);
       // robot.linSlideRight.setPower(-0.5);

//        robot.linSlideLeft.setTargetPosition(5);
//        robot.linSlideRight.setTargetPosition(5);

       /* while (robot.linSlideLeft.getCurrentPosition() > 45 && opModeIsActive()){
        }

        robot.linSlideLeft.setPower(0);
        robot.linSlideRight.setPower(0);
*/
//        pivot(10, 0.5);
//        move(5, 0.5);
//        pivot(10, -0.5);

    }

   /* public int getOrientation (){

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        if (initFlag == false){
            initVuforia();
            initFlag = true;
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

        }

//                 /** Wait for the game to begin */
//                 telemetry.addData(">", "Press Play to start tracking");
//                 telemetry.update();
//                 waitForStart();
/*
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
          /*  if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.update();
                        sleep(1000);

                            if (updatedRecognitions.size() == 1 || updatedRecognitions.size() == 0) {
                                //return(0);
                            }
                            /*else */ /*
                            if ((updatedRecognitions.size() == 2) && opModeIsActive()) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }

                                }
                                if (goldMineralX != -1 && silverMineral1X != -1) {
                                    if (goldMineralX < silverMineral1X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        telemetry.update();
                                        sleep(1000);
                                        return (1); //left
                                    } else if (goldMineralX > silverMineral1X) {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        telemetry.update();
                                        sleep(1000);
                                        return (2); //center
                                    }
                                } else if ((silverMineral1X != -1) && (silverMineral2X != -1)) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    sleep(1000);
                                    return (3); //right
                                } else {
                                    telemetry.addData("Gold Mineral Position", "WTF");
                                    telemetry.update();
                                    sleep(1000);
                                    return (10);
                                }
                            } else if ((updatedRecognitions.size() == 3) && opModeIsActive()) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
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
                                        telemetry.update();
                                        sleep(1000);
                                        return (1); //left
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {

                                        telemetry.addData("Gold Mineral Position", "Right");
                                        telemetry.update();
                                        sleep(1000);
                                        return (3); //right
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        telemetry.update();
                                        sleep(1000);
                                        return (2); //center
                                    }
                                }
                            }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
            //tfod.deactivate();
        }
        telemetry.addData("Failure", "Failure");
        telemetry.update();
        sleep(1000);
        return(0);

    }

    /***
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


//     public String getOrientationLegacy (){
//
//                 // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//                 // first.
//                 if (initFlag == false){
//                     initVuforia();
//                     initFlag = true;
//                     if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//                         initTfod();
//                     } else {
//                         telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//                     }
//
//                 }
//
////                 /** Wait for the game to begin */
////                 telemetry.addData(">", "Press Play to start tracking");
////                 telemetry.update();
////                 waitForStart();
//
//                 if (opModeIsActive()) {
//                     /** Activate Tensor Flow Object Detection. */
//                     if (tfod != null) {
//                         tfod.activate();
//                     }
//
//                     while (opModeIsActive()) {
//                         if (tfod != null) {
//                             // getUpdatedRecognitions() will return null if no new information is available since
//                             // the last time that call was made.
//                             List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                             if (updatedRecognitions != null) {
//                                 telemetry.addData("# Object Detected", updatedRecognitions.size());
//                                 if ((updatedRecognitions.size() == 1) && (updatedRecognitions.size() == 2)) {
//                                     return "insufficient";
//                                 } else if (updatedRecognitions.size() == 3) {
//                                     int goldMineralX = -1;
//                                     int silverMineral1X = -1;
//                                     int silverMineral2X = -1;
//                                     for (Recognition recognition : updatedRecognitions) {
//                                         if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                             goldMineralX = (int) recognition.getLeft();
//                                         } else if (silverMineral1X == -1) {
//                                             silverMineral1X = (int) recognition.getLeft();
//                                         } else {
//                                             silverMineral2X = (int) recognition.getLeft();
//                                         }
//                                     }
//                                     if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                         if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                             telemetry.addData("Gold Mineral Position", "Left");
//                                             return("left");
//                                         } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                             telemetry.addData("Gold Mineral Position", "Right");
//                                             return("right");
//                                         } else {
//                                             telemetry.addData("Gold Mineral Position", "Center");
//                                             return("center");
//                                         }
//                                     }
//                                 }
//                                 telemetry.update();
//                             }
//                         }
//                     }
//                 }
//
//                 if (tfod != null) {
//                     tfod.shutdown();
//                 }
//
//                 return("none");
//
//             }
//
//             /**
//              * Initialize the Vuforia localization engine.
//              */
//             private void initVuforia() {
//                 /*
//                  * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//                  */
//                 VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//                 parameters.vuforiaLicenseKey = VUFORIA_KEY;
//                 parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//                 //  Instantiate the Vuforia engine
//                 vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//                 // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//             }
//
//             /**
//              * Initialize the Tensor Flow Object Detection engine.
//              */
//             private void initTfod() {
//                 int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                         "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                 TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//                 tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//                 tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);


    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    //private VuforiaLocalizer vuforia = null;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public boolean getPosition(){
         boolean targetVisible = false;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

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
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;


                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    return true;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        return false;
    }}

//             }



