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

    double ChassisMotorEncoderCount = 723.24;

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
        //robotRadius = 10.93; //inches - Radius = half of longest diagonal = 0.5*sqrt(sq(17)+sq(18).
        robotRadius = 9.08; //Based on 3375 for accurate turn 3350* wheelRadius/EncoderCount Thomas
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
        setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // To avoid jerk at start
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
        frontLeft.setTargetPosition(0); //Thomas added
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(runMode);

        runMode = frontRight.getMode();
        frontRight.setTargetPosition(0); // Thomas added
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(runMode);

        runMode = backLeft.getMode();
        backLeft.setTargetPosition(0); //Thomas added
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(runMode);

        runMode = backRight.getMode();
        backRight.setTargetPosition(0); // Thomas added
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

    /**
     * Method to move chassis based on computed vector inputs for a set distance.
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param distance in same unit of measure as wheelRadius
     * @param targetAngle 0 for forward motion -PI/2 for straving right, + PI/2 for straving left, + PI for backward
     * @param turn Turn angle PI/2 for clockwise 90 degrees -PI/2 for anticlockwise 90 degrees
     * @param power Power to be used to run motors.
     */
    public void runDistance(double distance, double targetAngle, double turn, double power) {
        //ChassisMotionTimeOut.reset(); // To protect against uncontrolled runs.
        double turnAngle = targetAngle + Math.PI / 4;

        double wheelDistance = distance * ChassisMotorEncoderCount/(2*Math.PI*wheelRadius);
        double robotTurn = robotRadius * turn * ChassisMotorEncoderCount/(2*Math.PI*wheelRadius);


        //Distribute power to wheels a cos and sin of vector.
        // Add turn as input from right stick to add in radiants
        frontLeft.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) + robotTurn));
        frontRight.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) - robotTurn));
        backLeft.setTargetPosition((int) (wheelDistance * Math.sin(turnAngle) + robotTurn));
        backRight.setTargetPosition((int) (wheelDistance * Math.cos(turnAngle) - robotTurn));

        //Run the Motors
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //To avoid jerk
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


        /**
     * Method to move chassis based on computed vector inputs for a set max_stop_distance
     * Till frontleftChassisTouchSensor is pressed.
     * To be used in Autonomous mode for moving by distance or turning by angle
     * Uses PID loop in motors to ensure motion without errors
     * @param max_stop_distance in same unit of measure as wheelRadius
     * @param power
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
        };
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
     * @param max_stop_distance
     * @param strave 0 for forward or backward, 1 for right, -1 for left
     * @param power
     */
    public void runFwdStraveTill_ChassisRightColorSensorIsRed(double max_stop_distance, double strave, double power){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        while (!rightColorSensorIsRed() && (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))) {
            if(strave==0) {
                //Go forward or backward
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            } else {/* #TOBEFIXED FOR STRAVE LOGIC
                frontLeft.setPower(power * Math.cos(3*Math.PI/4));
                frontRight.setPower(power * Math.sin(3*Math.PI/4));
                backLeft.setPower(power * Math.sin(Math.PI/4));
                backRight.setPower(power * Math.cos(Math.PI/4));
            */}
        };
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
    //Once completed replicate for other 3 combinations Right-Blue, Left-Red, Left-Blue

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
    }

}
