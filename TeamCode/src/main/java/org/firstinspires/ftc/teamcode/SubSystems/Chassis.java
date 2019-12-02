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
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisTeleOpMethods : runByGamepadCommand()
 * @ChassisAutoMethods : runDistance()
 * @ChassisAutoMethods : runTill_frontleftBumperSensor_Pressed(max stop distance)
 * @ChassisAutoMethods : runTill_chassisLocationSensorIdentifiesLine(color)
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
    public TouchSensor frontleftChassisTouchSensor; //Analog mode of Touch Sensor


    //Declare Color Sensors
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;

    //Declare Chassis Configuration variables
    public double wheelRadius;
    public double robotRadius;

    //Timer for timing out Arm motion incase targetPosition cannot be achieved
    ElapsedTime ChassisMotionTimeOut = new ElapsedTime();

    public boolean configureRobot = false;

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
        frontleftChassisTouchSensor = hardwareMap.touchSensor.get("ch_touch_sensor"); //Analog mode
        //frontleftChassisTouchSensor = hardwareMap.get(DigitalChannel.class, "ch_touch_sensor");

        //Map ColorSensors from configuration
        leftColorSensor = hardwareMap.get(ColorSensor.class, "ch_left_color");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "ch_right_color");

        //Configure Robot to dimensions and modified for wheel type
        configureRobot();

        //Initializes Robot to right component modes - Reset, Set Zero Behavior
        initChassis();
    }

    /**
     * Configure Chassis for size and mecanum wheel directions
     */
    public void configureRobot(){
        wheelRadius = 1.965; //inches
        // Robot width = 17.00 inch, length = 13.75 inch. Hypotensuse = 21.86 inch, radius = hyp/2 = 10.93
        robotRadius = 10.93; //inches - Radius = half of longest diagonal = 0.5*sqrt(sq(17)+sq(18).

        //Set direction of motors wrt motor drive set up, so that wheels go forward +y power
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        configureRobot = true;
    }


    /**
     * Initialize Chassis to right component modes - Reset, Set Zero Behavior
     */
    public void initChassis() {
        resetChassis();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the digital channel Touch Sensor to input.
        //frontleftChassisTouchSensor.setMode(DigitalChannel.Mode.INPUT);
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
        //Logic to detect Red R>200 G<127 B<127
        if (leftColorSensor.red()>200 && leftColorSensor.green()<127 && leftColorSensor.blue()<127 && leftColorSensor.alpha()>60) {
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
        if (rightColorSensor.red()>200 && rightColorSensor.green()<127 && rightColorSensor.blue()<127 && rightColorSensor.alpha()>60) {
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
        //Logic to detect Blue R<127 G<127 B>200
        if (rightColorSensor.red()<127 && rightColorSensor.green()<127 && rightColorSensor.blue()>200 && rightColorSensor.alpha()>60) {
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
        //Logic to detect Blue R<127 G<127 B>200
        if (leftColorSensor.red()<127 && leftColorSensor.green()<127 && leftColorSensor.blue()>200 && leftColorSensor.alpha()>60) {
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
        //#TOBEFILLED Why subtract by 90dec?
        double turnAngle = targetAngle - Math.PI / 4;

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
        ChassisMotionTimeOut.reset();
        double turnAngle = targetAngle + Math.PI / 4;
        //double wheelDistance = (Math.sqrt(2) / wheelRadius) * distance;
        double wheelDistance = 1440*distance/(2*Math.PI*wheelRadius);
        double robotTurn = robotRadius * turn *1440;

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
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        //wait till motor finishes motion
        //while(frontLeft.isBusy() && ChassisMotionTimeOut.milliseconds() < 10000);

    }

    /**
     * Method to move chassis by rotation.
     * Used in Auto placement of block
     * @param rotations
     * @param power
     */
    public void runRotations(double rotations, double power) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetChassis();

        //#TOBEFILLED A US Digital E4P encoder (in the Tetrix kit) will report 1440 counts for one revolution of the motor shaft
        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(1440 * rotations)) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        };
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to move chassis by rotation.
     * Used in Auto placement of block
     * @param distance
     * @param power
     */
    public void runStraightDistanceByRotations(double distance, double power) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = distance/(2*Math.PI*wheelRadius);

        //#TOBEFILLED A US Digital E4P encoder (in the Tetrix kit) will report 1440 counts for one revolution of the motor shaft
        while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(1440 * targetRotations)) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        };
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
     * @param targetAngle
     * @param turn
     * @param power
     */
    public void runTill_frontleftChassisTouchSensor_Pressed(double max_stop_distance, double targetAngle, double turn, double power) {
        //#TOBEFILLED
        double turnAngle = targetAngle - Math.PI / 4;double wheelDistance = (Math.sqrt(2) / wheelRadius) * max_stop_distance;
        double robotTurn = robotRadius * turn;

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
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    /**
     * Method to turn robot by a specified angle.
     */
    public void turnRobotByAngle(double robotTurn, double power) {
        //#TOBEFILLED
    }
    /**
     * Method to identify when frontleftChassisTouchSensor is pressed
     * frontleftChassisTouchSensor.getState() return true when not touched
     *
     */
    public boolean frontleftChassisTouchSensorIsPressed(){
        /* Analog Mode */
        if (frontleftChassisTouchSensor.isPressed()){

            //function returns tr
            return true;
        } else {
            return false;
        }


        /*
        if (frontleftChassisTouchSensor.getState()){
                     //function returns true when not touched, so isPressed is false
            return false;
        } else {
            //function returns false when touched, so isPressed is true
            return true;
        }
        */
    }

}
