package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 1
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone and
 *                  park near wall or near neutral Skybridge
 *
 * Steps:
 * Robot starts on SB5
 * On start, robot opens wrist to front position
 * Lift Arm to Sense Position using Arm Color Sensor
 * If Skystone, record Skystone position as SB5, Go to Step 10
 * Else move robot to SB4. Check on Arm Color Sensor for Skystone.
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

@Autonomous(name = "BLUE-SkyStone-ParkBridge", group = "Autonomous")
public class BLUE_SkyStone_ParkBridge extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    int skystonePosition;

    public int robotDepth = 17; // Ball on wall to Edge of Chassis Touch sensor
    public int robotWidth = 17; // Wheel edge to wheel edge

    int playingAlliance = 1; //1 for Blue, -1 for Red
    boolean parkingPlaceNearSkyBridge = false;//false for near wall, true for near NeutralSkybridge

    boolean parked = false; // Will be true once robot is parked

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

        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        //Robot starts on A2
        waitForStart();
        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        while (!isStopRequested() && !parked) {
            AutonomousUC1Commands();
        }
    }

    void AutonomousUC1Commands() {

        // Robot starts on SB5

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        // Lift Arm to Sense Position

        sleep(1000);
        moveArm_groundLevel();
        sleep(1000);
        moveArm_detectSkystoneLevel();
        sleep(1000);
        // Move by distance X forward near SB5 : 6 inches to skystone
        double robotToNearSkystone = 20;
        runFwdBackLeftRight(robotToNearSkystone,0,0.1);

        sleep(100);

        // Check on color sensor, for Skystone
        moveTillStoneDetected();
        sleep(1000);
        // If Skystone, record Skystone position as SB5, Go to Step 10

        skystonePosition = 5; // Assume current position is skystone
        double stoneTostone = 8;
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

        // Drop Arm and Grip the block.
        moveArm_groundLevel();
        sleep(1000);
        closeGrip();

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
        autoArm.armMotor.setTargetPosition(autoArm.detectSkystoneLevel);
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



}
