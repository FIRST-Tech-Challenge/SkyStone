package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Autonomous Mode Usecase 1
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone and
 *                  park near wall or near neutral Skybridge
 *
 * Steps:
 * Robot starts on SB5
 * On start, robot opens wrist to front position
 * Lift Arm to Sense Position using Tensor Flow
 * If Skystone, record Skystone position as SB5, Go to Step 10
 * Else move robot to SB4. Check on Tensor Flow for Skystone.
 * If Skystone, record Skystone position as SB4, Go to Step 10
 * Else move robot to SB3. record skystone position as SB3.
 * Grip and pick the block.
 * Lift Arm to Level 2 Tray Height
 * Slide back to edge of B2,
 * Turn 90 degrees Left
 * Move to B4
 * Drop block
 * Move in between B4 and B3 (Parking)
 *
 *  Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance
 *  Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge
 */

@Autonomous(name = "BLUE-SkyStone-ParkBridge-TensorFlow", group = "Autonomous")
public class BLUE_SkyStone_ParkBridge_TensorFlow extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    int skystonePosition;

    public int robotDepth = 17; // Ball on wall to Edge of Chassis Touch sensor
    public int robotWidth = 17; // Wheel edge to wheel edge

    int playingAlliance = 1; //1 for Blue, -1 for Red
    boolean parkingPlaceNearSkyBridge = false;//false for near wall, true for near NeutralSkybridge

    boolean parked = false; // Will be true once robot is parked

    //*************Vuforia******************
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private double x_translate = 0;
    private double y_translate = 0;
    double stoneTostone = 8;

    public VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    int cameraMonitorViewId;

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT  = -5.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 12.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    //*************Vuforia******************

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    /**
     * Template runOpMode code. Only change Usecase function and call here.
     * Refer to Autonomous Command Syntax to put right values
     * <p>
     * All Usecases written assuming playingAlliance = 1 meaning Blue, -1 for Red.
     *
     * throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        vuforiaInit();

        telemetry.setAutoClear(false);
        telemetry.addData("before wait for start","");
        telemetry.update();

        //Robot starts on A2
        waitForStart();
        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        //*************Vuforia************************
        targetsSkyStone.activate();
        //*************Vuforia************************

        telemetry.addData("before loop","");
        telemetry.update();
        while (!isStopRequested() && !parked) {
            AutonomousUC1Commands();
        }
    }

    void AutonomousUC1Commands() {

        // Robot starts on SB5

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        // Lift Arm to Sense Position

        sleep(500);
        moveArm_groundLevel();
        openGrip();
        sleep(500);
        moveArm_detectSkystoneLevel();
        sleep(500);

        // Move by distance X forward near SB5 : 6 inches to skystone
        double robotToNearSkystone = 26;
        runFwdBackLeftRight(robotToNearSkystone,0,0.1);

        sleep(1000);

        telemetry.addData("before viewforia in loop","");
        telemetry.update();

        skystonePosition = 5;
        vuforiaFindSkystone();
        sleep (1000);
        telemetry.addData("after viewforia: Target visible: ",targetVisible);
        telemetry.update();

        if (targetVisible){
            if(Math.abs(x_translate)>1){
                runFwdBackLeftRight(Math.abs(x_translate), (int) x_translate/Math.abs(x_translate),0.1);
            }
        }
        sleep(500);
        /*if (targetVisible){
            runFwdBackLeftRight(y_translate, 0,0.1);
        }*/

        //If target is not visible move to next block
        if(!targetVisible) {
            runFwdBackLeftRight(stoneTostone, playingAlliance, 0.1);
            skystonePosition = 4;
        }

        if(!targetVisible && (skystonePosition == 4)){
                vuforiaFindSkystone();
        }

        if (targetVisible && (skystonePosition == 4)){
            runFwdBackLeftRight(Math.abs(x_translate), x_translate/Math.abs(x_translate),0.1);
        }
        /*sleep(500);
        if (targetVisible){
            runFwdBackLeftRight(y_translate, 0,0.1);
        }*/

        //If target is not visible move to next block
        if(!targetVisible && (skystonePosition == 4)) {
            runFwdBackLeftRight(stoneTostone, playingAlliance, 0.1);
            skystonePosition = 3;
        }


        /*****Commented for Vuforia
        // Check on color sensor, for Skystone
        moveTillStoneDetected();
        sleep(1000);
        // If Skystone, record Skystone position as SB5, Go to Step 10

        skystonePosition = 5; // Assume current position is skystone

        if ((autoIntake.stoneDetected) && (!autoIntake.skystoneDetected)) {
            //Skystone not detected, move to SB4
            skystonePosition = 4;
            runFwdBackLeftRight(stoneTostone,playingAlliance,0.1);
        }
        sleep(1000);

        // Check on color sensor, for Skystone
        moveTillStoneDetected();
        sleep(1000);

        if ((autoIntake.stoneDetected) && (!autoIntake.skystoneDetected)) {
            //Skystone not detected, move to SB3
            skystonePosition = 3;
            runFwdBackLeftRight(stoneTostone,playingAlliance,0.1);
        }
        sleep(1000);
         Commented for Vuforia************/

        // Drop Arm and Grip the block.
        moveArm_groundLevel();
        sleep(1000);
        closeGrip();
        /*
        // Slide back to edge of B2, 10 inches
        runFwdBackLeftRight(-10,0,0.1);

        sleep(1000);
        // Turn 90 degrees Left
        turnby90degree(playingAlliance*(-1),0.1);
        sleep(1000);


        //Lift Arm
        moveArm_aboveFoundationLevel();

       //Move forward till Chassis bumber limit switch is pressed.
        double expectedMaxDistanceToFoundation = 40;
        runFwdTill_frontleftChassisTouchSensor_Pressed(expectedMaxDistanceToFoundation, 0.1);

        // Drop block
        openGrip();

        // Move in between B4 and B3 (Parking)
        // Park near wall
        // Move back by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
        if (playingAlliance == 1) {
            runTill_ChassisRightColorSensorIsBlue(-55, 0, 0.25);
        } else {
            runTill_ChassisLeftColorSensorIsRed(-55, 0, 0.25);
        }
        */

        parked = true;
        //End of Usecase : Should be parked at this time.
    }


    /**
     * Method to move till Skystone is detected using color sensor and distance sensor
     */
    void moveTillStoneDetected(){
        //public void runTill_ChassisLeftColorSensorIsBlue(double max_stop_distance, double straveDirection, double power){

        double stoneDetect_max_stop_distance = 6; //max is 6"
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = stoneDetect_max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        while (!isStopRequested() && !autoIntake.detectSkytoneAndType() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            autoChassis.frontLeft.setPower(0.1);
            autoChassis.frontRight.setPower(0.1);
            autoChassis.backLeft.setPower(0.1);
            autoChassis.backRight.setPower(0.1);
        }

        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);

    } //return stone detected autoIntake.stoneDetected and if skystone autoIntake.SkystoneDetected

    /* Method to move chassis based on computed vector inputs for a set distance
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     *
     * @param distance +ve for forward, -ve for backward
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runFwdBackLeftRight(double distance, double strafeDirection, double power){
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = distance/(2*Math.PI*autoChassis.wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = distance /Math.abs(distance);

        while (!isStopRequested() && (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                autoChassis.frontLeft.setPower(fwdbackdirection*power);
                autoChassis.frontRight.setPower(fwdbackdirection*power);
                autoChassis.backLeft.setPower(fwdbackdirection*power);
                autoChassis.backRight.setPower(fwdbackdirection*power);
            } else {
                autoChassis.frontLeft.setPower(strafeDirection* power);
                autoChassis.frontRight.setPower(-strafeDirection* power);
                autoChassis.backLeft.setPower(-strafeDirection* power);
                autoChassis.backRight.setPower(strafeDirection* power);
            }
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till frontleftChassisTouchSensor is pressed.
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance in same unit of measure as wheelRadius
     * @param power to run motors
     */
    public void runFwdTill_frontleftChassisTouchSensor_Pressed(double max_stop_distance, double power) {
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        while (!isStopRequested() && !autoChassis.frontleftChassisTouchSensor.isPressed() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            autoChassis.frontLeft.setPower(power);
            autoChassis.frontRight.setPower(power);
            autoChassis.backLeft.setPower(power);
            autoChassis.backRight.setPower(power);
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move Arm to onFoundationLevel and turn Brake Mode ON
     */
    public void moveArm_onFoundationLevel(){
        autoArm.armMotor.setTargetPosition(autoArm.onFoundationLevel);
        autoArm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autoArm.runArmToLevel();
    }

    /**
     * Method to move Arm to aboveFoundationLevel and turn Brake Mode ON
     */
    public void moveArm_aboveFoundationLevel(){
        autoArm.armMotor.setTargetPosition(autoArm.aboveFoundationLevel);
        turnArmBrakeModeOn();
        autoArm.runArmToLevel();
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors at
     */
    public void runTill_ChassisRightColorSensorIsRed(double max_stop_distance, double strafeDirection, double power){
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!isStopRequested() && !autoChassis.rightColorSensorIsRed() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                autoChassis.frontLeft.setPower(fwdbackdirection*power);
                autoChassis.frontRight.setPower(fwdbackdirection*power);
                autoChassis.backLeft.setPower(fwdbackdirection*power);
                autoChassis.backRight.setPower(fwdbackdirection*power);
            } else {
                autoChassis.frontLeft.setPower(strafeDirection* power);
                autoChassis.frontRight.setPower(-strafeDirection* power);
                autoChassis.backLeft.setPower(-strafeDirection* power);
                autoChassis.backRight.setPower(strafeDirection* power);
            }
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move wrist to Initial position
     */
    public void moveWristToClose() {
        autoIntake.wrist.setPosition(autoIntake.wristPosition[0]);//close position = 0.2
        autoIntake.wristCurrentPosition = 0;
    }

    /**
     * Method to set Arm brake mode to ON when Zero (0.0) power is applied.
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn(){
        autoArm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autoArm.armMotor.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors at
     */
    public void runTill_ChassisRightColorSensorIsBlue(double max_stop_distance, double strafeDirection, double power){
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!isStopRequested() && !autoChassis.rightColorSensorIsBlue() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                autoChassis.frontLeft.setPower(fwdbackdirection*power);
                autoChassis.frontRight.setPower(fwdbackdirection*power);
                autoChassis.backLeft.setPower(fwdbackdirection*power);
                autoChassis.backRight.setPower(fwdbackdirection*power);
            } else {
                autoChassis.frontLeft.setPower(strafeDirection* power);
                autoChassis.frontRight.setPower(-strafeDirection* power);
                autoChassis.backLeft.setPower(-strafeDirection* power);
                autoChassis.backRight.setPower(strafeDirection* power);
            }
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors at
     */
    public void runTill_ChassisLeftColorSensorIsRed(double max_stop_distance, double strafeDirection, double power){
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!isStopRequested() && !autoChassis.leftColorSensorIsRed() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                autoChassis.frontLeft.setPower(fwdbackdirection*power);
                autoChassis.frontRight.setPower(fwdbackdirection*power);
                autoChassis.backLeft.setPower(fwdbackdirection*power);
                autoChassis.backRight.setPower(fwdbackdirection*power);
            } else {
                autoChassis.frontLeft.setPower(strafeDirection* power);
                autoChassis.frontRight.setPower(-strafeDirection* power);
                autoChassis.backLeft.setPower(-strafeDirection* power);
                autoChassis.backRight.setPower(strafeDirection* power);
            }
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors at
     */
    public void runTill_ChassisLeftColorSensorIsBlue(double max_stop_distance, double strafeDirection, double power){
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!isStopRequested() && !autoChassis.leftColorSensorIsBlue() &&
                (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                autoChassis.frontLeft.setPower(fwdbackdirection*power);
                autoChassis.frontRight.setPower(fwdbackdirection*power);
                autoChassis.backLeft.setPower(fwdbackdirection*power);
                autoChassis.backRight.setPower(fwdbackdirection*power);
            } else {
                autoChassis.frontLeft.setPower(strafeDirection* power);
                autoChassis.frontRight.setPower(-strafeDirection* power);
                autoChassis.backLeft.setPower(-strafeDirection* power);
                autoChassis.backRight.setPower(strafeDirection* power);
            }
        }
        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    /**
     * Method to move Arm to groundlevel and turn Brake Mode OFF
     */
    public void moveArm_groundLevel(){
        autoArm.armMotor.setTargetPosition(autoArm.groundLevel);
        autoArm.turnArmBrakeModeOff();
        autoArm.runArmToLevel();
        //resetArm();
    }

    /**
     * Method to move Arm to detectSkystoneLevel and turn Brake Mode ON
     */
    public void moveArm_detectSkystoneLevel(){
        //**************Vuforia************
        autoArm.armMotor.setTargetPosition(-200);
        //**************Vuforia************
        //autoArm.armMotor.setTargetPosition(autoArm.detectSkystoneLevel);
        turnArmBrakeModeOn();
        autoArm.runArmToLevel();
    }

    /**
     * Method to open Grip
     */
    public void openGrip() {
        autoIntake.grip.setPosition(autoIntake.gripOpenPosition);
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip() {
            autoIntake.grip.setPosition(autoIntake.gripClosePosition);
    }

    /**
     * Method to turn robot by 90 degrees
     * @param clockOrAntiClockwise + 1 for clockwise, -1 for anticlockwise
     * @param power for motors
     *
     */
    public void turnby90degree(int clockOrAntiClockwise, double power){
        autoChassis.resetChassis();
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Max Total Rotations of wheel = distance / circumference of wheel
        //double target90degRotations = (Math.PI*robotRadius/2)/(2*Math.PI*wheelRadius);


        while (!isStopRequested() && Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * autoChassis.target90degRotations)) {
            autoChassis.frontLeft.setPower(clockOrAntiClockwise*power);
            autoChassis.frontRight.setPower(-clockOrAntiClockwise*power);
            autoChassis.backLeft.setPower(clockOrAntiClockwise*power);
            autoChassis.backRight.setPower(-clockOrAntiClockwise*power);
        }
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);
    }

    public void vuforiaInit(){
        //*************Vuforia******************
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        VectorF translation;

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        //*************Vuforia************************

    }

    public void vuforiaFindSkystone(){
        AutonomousTimeOut.startTime();
        targetVisible = false;
        while(AutonomousTimeOut.milliseconds()<10000 && !targetVisible) {
            //**************Vuforia********************
            // check all the trackable targets to see which one (if any) is visible.
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
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                x_translate = translation.get(0)/mmPerInch;
                y_translate = translation.get(1) / mmPerInch;

            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        }

        targetsSkyStone.deactivate();
        //***********Vuforia**************

    }


}
