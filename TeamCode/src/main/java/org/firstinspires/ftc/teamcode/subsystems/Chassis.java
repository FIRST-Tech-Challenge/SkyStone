package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *  Definition of Robot Chassis.
 *  Chassis has :
 *      4 DC motors connected to Mecanum wheels
 *      1 limit switch on left front bumper to identify hitting to foundation plate or other walls
 *      1 light sensor pointing down (to identify red / blue lines below skybridge for parking
 *
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisMethods : runByGamepadCommand()
 * @ChassisMethods : runDistance()
 * @ChassisMethods : runTill_frontleftBumperSensor_Pressed(max stop distance)
 * @ChassisMethods : runTill_chassisLocationSensorIdentifiesLine(color)
 *
 */

/**
 * Class Definition
 */
public class Chassis {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public Double wheelRadius;
    public Double robotRadius;
    //Declare TouchSensor frontleftBumperSensor #TOBEFILLED
    //Declare LightSensor chassisLocationSensor #TOBEFILLED

    public Chassis(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("front_left_drive");
        frontRight = hardwareMap.dcMotor.get("front_right_drive");
        backLeft = hardwareMap.dcMotor.get("back_left_drive");
        backRight = hardwareMap.dcMotor.get("back_right_drive");
        //frontleftBumperSensor = #TOBEFILLED
        //chassisLocationSensor = #TOBEFILLED
        initChassis();
    }

    /**
     * Configure Chassis #TOBEFILLED
     */
    //wheelRadius =
    //robotRadius =

    /**
     * Initialize Chassis - Reset, Set Zero Behavior
     */
    public void initChassis() {
        resetChassis();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //setfrontleftBumperSensorMode = #TOBEFILLED
        //frontleftBumperSensorMode = #TOBEFILLED
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder.
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for the motor
     */
    public void resetChassis() {

        DcMotor.RunMode runMode = frontLeft.getMode();
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(runMode);

        runMode = frontRight.getMode();
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(runMode);

        runMode = backLeft.getMode();
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(runMode);

        runMode = backRight.getMode();
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(runMode);

        //frontleftBumperSensor = #TOBEFILLED
        //chassisLocationSensor = #TOBEFILLED


    }

    /**
     * Function to set the behaviour of the motor on passing Zero power to the motor
     * @param zeroPowerBehavior could be BRAKE or FLOAT. When not defined, it is set
     *                          to UNKNOWN state, which is not desired.
     */
    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity
     * RUN_USING_ENCODER (run at a targeted velocity or RUN_TO_POSITION (PID based rotation to
     * achieve the desited encoder count
     * @param runMode
     */
    public void setMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    /**
     * Method to move chassis based on computed vector inputs from Gamepad Joystick inputs
     * @param targetAngle targetAngle = Math.atan2(leftStickY, leftStickX)
     * @param turn turn = rightStickX
     * @param power power = Math.hypot(leftStickX, leftStickY)
     */
    public void runByGamepadCommand(double targetAngle, double turn, double power) {
        //#TOBEFILLED Why subtract by 90dec?
        final double turnAngle = targetAngle - Math.PI / 4;

        //Distribute power to wheels a cos and sin of vector.
        // Add turn as input from right stick to add in radiants
        frontLeft.setPower(power * Math.cos(turnAngle) + turn);
        frontRight.setPower(power * Math.sin(turnAngle) - turn);
        backLeft.setPower(power * Math.sin(turnAngle) + turn);
        backRight.setPower(power * Math.cos(turnAngle) - turn);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set distance.
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param distance in same unit of measure as wheelRadius
     * @param targetAngle
     * @param turn
     * @param power
     */
    public void runDistance(double distance, double targetAngle, double turn, double power) {
        //#TOBEFILLED
        final double turnAngle = targetAngle - Math.PI / 4;
        final double wheelDistance = (Math.sqrt(2) / wheelRadius) * distance;
        final double robotTurn = robotRadius * turn;

        //#TOBEFILLED
        frontLeft.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) + robotTurn));
        frontRight.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) - robotTurn));
        backLeft.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) + robotTurn));
        backRight.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) - robotTurn));

        //#TOBEFILLED
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Method to move chassis by rotation.
     * Used in Auto placement of block
     * @param rotations
     * @param power
     */
    public void runRotations(double rotations, double power) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        //#TOBEFILLED
        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(1440 * rotations)) ;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till frontleftBumperSensor is pressed.
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance in same unit of measure as wheelRadius
     * @param targetAngle
     * @param turn
     * @param power
     */
    public void runTill_frontleftBumperSensor_Pressed(double max_stop_distance, double targetAngle, double turn, double power) {

    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance
     * @param targetAngle
     * @param turn
     * @param power
     */
    public void runTill_chassisLocationSensorIdentifiesLine(double max_stop_distance, double targetAngle, double turn, double power){
        //Color needs to be added to definition
    }


}
