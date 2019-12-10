package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Definition of Robot Chassis.
 *  Chassis has :
 *      4 DC motors connected to Mecanum wheels
 *      1 limit switch on left front bumper to identify hitting to foundation plate or other walls
 *      2 Color sensors pointing down one on left and another on right
 *      (to identify red / blue lines below skybridge for parking
 *
 *      Robot 1 : Chassis Motor : 5201 Series, 26:1 Ratio, 210 RPM Spur Gear Motor w/Encoder
 *      Encoder Countable Events Per Revolution (Output Shaft)	723.24 (Rises & Falls of Ch A & B)
 *
 *      Robot 2 : 5202 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder)
 *      Encoder Countable Events Per Revolution (Output Shaft)	537.6 (Rises & Falls of Ch A & B)
 *
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisTeleOpMethods : runByGamepadCommand()
 * @ChassisAutoMethods : runDistance()
 * @ChassisAutoMethods : runFwdTill_frontleftChassisTouchSensor_Pressed(
 * @ChassisAutoMethods : runTill_ChassisRightColorSensorIsRed()
 * @ChassisAutoMethods : turnRobotByAngle()
 * @ChassisAutoMethods : resetColorSensorEnabled()
 * @ChassisAutoMethods : leftColorSensorIsRed()
 * @ChassisAutoMethods : rightColorSensorIsBlue()
 * @ChassisAutoMethods : leftColorSensorIsBlue()
 * @ChassisAutoMethods : rightColorSensorIsRed()
 * @ChassisAutoMethods : frontleftBumperSensorIsPressed()
 *
 */

public class Chassis {

    //Declare Chassis Motor and configurations
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Declare TouchSensor on front left of Chassis
    public TouchSensor frontleftChassisTouchSensor;

    //Declare Color Sensors
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;

    //Declare Chassis Configuration variables
    public double wheelRadius;
    public double robotRadius;
    public double target90degRotations;

    //Timer for timing out Arm motion incase targetPosition cannot be achieved
    ElapsedTime ChassisMotionTimeOut = new ElapsedTime();

    public boolean configureRobot = false;

    public double ChassisMotorEncoderCount = 723.24;

    /**
     * Constructor of Chassis.
     * Sets hardwareMap on hub1 with 4 motors, 2 light sensors, 1 touch sensors
     * Configures Robot for size and mecanum wheel directions
     * Initialize Robot to right component modes.
     * @param hardwareMap HardwareMap to be setup on Hub1
     */
    public Chassis(HardwareMap hardwareMap) {
        //Map DCMotors from configuration
        frontLeft = hardwareMap.dcMotor.get("front_left_drive");
        frontRight = hardwareMap.dcMotor.get("front_right_drive");
        backLeft = hardwareMap.dcMotor.get("back_left_drive");
        backRight = hardwareMap.dcMotor.get("back_right_drive");

        //Map TouchSensor from configuration
        frontleftChassisTouchSensor = hardwareMap.touchSensor.get("ch_touch_sensor");

        //Map ColorSensors from configuration
        leftColorSensor = hardwareMap.get(ColorSensor.class, "ch_left_color");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "ch_right_color");

        //Configure Robot to dimensions and modified for wheel type
        configureRobot();
    }

    /**
     * Configure Chassis for size and mecanum wheel directions
     */
    public void configureRobot(){
        wheelRadius = 1.965*7/5; //100mm and applied correction factor of 7/5 for slippage
        robotRadius = 9.3*7/5; //Radius = half of longest diagonal = 0.5*sqrt(sq(14.5)+sq(10.5).and applied correction factor of 7/5 for slippage
        //Set direction of motors wrt motor drive set up, so that wheels go forward +y power

        target90degRotations = (Math.PI*robotRadius/2)/(2*Math.PI*wheelRadius);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        configureRobot = true;
    }


    /**
     * Initialize Chassis to right component modes - Reset, Set Zero Behavior
     */
    public void initChassis() {
        resetChassis();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // To avoid jerk at start
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder.
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for the motor
     *
     * Reset Color Sensors to off for TeleOpMode
     */
    public void resetChassis() {

        DcMotor.RunMode runMode = frontLeft.getMode();
        frontLeft.setTargetPosition(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(runMode);

        runMode = frontRight.getMode();
        frontRight.setTargetPosition(0);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(runMode);

        runMode = backLeft.getMode();
        backLeft.setTargetPosition(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(runMode);

        runMode = backRight.getMode();
        backRight.setTargetPosition(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(runMode);

        leftColorSensor.enableLed(false);
        rightColorSensor.enableLed(false);
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
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity)
     * RUN_USING_ENCODER (run at a targeted velocity) or RUN_TO_POSITION (PID based rotation to
     * achieve the desited encoder count)
     * @param runMode RUN_WITHOUT_ENCODER
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    /**
     * Method to set the ColorSensor to be enabled or disabled
     * @param colorSensorEnabled to set the mode of color sensor on
     */
    public void setLeftColorSensorEnabled(boolean colorSensorEnabled){
        leftColorSensor.enableLed(colorSensorEnabled);
    }

    /**
     * Method to set the ColorSensor to be enabled or disabled
     * @param colorSensorEnabled to set the mode of color sensor on
     */
    public void setRightColorSensordEnabled(boolean colorSensorEnabled){
             rightColorSensor.enableLed(colorSensorEnabled);
    }


    /**
     * Method to check for left Color Sensor crossing over Red Line
     * Used in Autonomous mode to stop below Red Skybridge after moving foundation to wall.
     * @return if Color Sensor is red
     */
    public boolean leftColorSensorIsRed() {
        //Logic to detect Red R>400
        if (leftColorSensor.red()>400) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for right Color Sensor crossing over Red Line
     * Used in Autonomous mode to stop below Red Skybridge after moving blocks (Optional use)
     * @return if color sensor is red
     */
    public boolean rightColorSensorIsRed() {
        //Logic to detect Red R>200 G<127 B<127
        if (rightColorSensor.red()>400){
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for right Color Sensor crossing over Blue Line
     * Used in Autonomous mode to stop below Red Skybridge after moving foundation to wall.
     * @return if color sensor is blue
     */
    public boolean rightColorSensorIsBlue() {
        //Logic to detect Blue B>400
        if (rightColorSensor.blue()>400) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for right Color Sensor crossing over Blue Line
     * Used in Autonomous mode to stop below Red Skybridge after after moving blocks (Optional use)
     * @return if color sensor is blue
     */
    public boolean leftColorSensorIsBlue() {
        //Logic to detect Blue B>400
        if (leftColorSensor.blue()>400) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to move chassis based on computed vector inputs from Gamepad Joystick inputs
     * @param targetAngle targetAngle = Math.atan2(leftStickY, leftStickX)
     * @param turn turn = rightStickX
     * @param power power = Math.hypot(leftStickX, leftStickY)
     */
    public void runByGamepadCommand(double targetAngle, double turn, double power) {
        //Rotate angle by 45 degrees to align to diagonal angles on mecannum wheel setup
        double turnAngle = targetAngle - Math.PI / 4;

        //Distribute power to wheels a cos and sin of vector.
        // Add turn as input from right stick to add in radiants
        frontLeft.setPower(power * Math.cos(turnAngle) + turn);
        frontRight.setPower(power * Math.sin(turnAngle) - turn);
        backLeft.setPower(power * Math.sin(turnAngle) + turn);
        backRight.setPower(power * Math.cos(turnAngle) - turn);
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
    }

    /* Method to move chassis based on computed vector inputs for a set distance
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     *
     * @param distance +ve for forward, -ve for backward
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runFwdBackLeftRight(double distance, double strafeDirection, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = distance /Math.abs(distance);

        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations)) {
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
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
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        while (!frontleftChassisTouchSensor.isPressed() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runTill_ChassisLeftColorSensorIsRed(double max_stop_distance, double strafeDirection, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!leftColorSensorIsRed() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runTill_ChassisLeftColorSensorIsBlue(double max_stop_distance, double strafeDirection, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!leftColorSensorIsBlue() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runTill_ChassisRightColorSensorIsRed(double max_stop_distance, double strafeDirection, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!rightColorSensorIsRed() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till team color is identified below Chassis
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     */
    public void runTill_ChassisRightColorSensorIsBlue(double max_stop_distance, double strafeDirection, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!rightColorSensorIsBlue() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to turn robot by 90 degrees
     * @param clockOrAntiClockwise + 1 for clockwise, -1 for anticlockwise
     * @param power to run motors
     */
    public void turnby90degree(int clockOrAntiClockwise, double power){
        resetChassis();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * target90degRotations)) {
            frontLeft.setPower(clockOrAntiClockwise*power);
            frontRight.setPower(-clockOrAntiClockwise*power);
            backLeft.setPower(clockOrAntiClockwise*power);
            backRight.setPower(-clockOrAntiClockwise*power);
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to identify when frontleftChassisTouchSensor is pressed
     * frontleftChassisTouchSensor.getState() return true when not touched
     *
     */
    public boolean frontleftChassisTouchSensorIsPressed(){
        if (frontleftChassisTouchSensor.isPressed()){
            return true;
        } else {
            return false;
        }
    }

}
