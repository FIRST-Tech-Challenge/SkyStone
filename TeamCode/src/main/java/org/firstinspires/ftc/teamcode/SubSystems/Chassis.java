package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Definition of Robot Chassis. <BR>
 *  Chassis has : <BR>
 *      4 DC motors connected to Mecanum wheels <BR>
 *      1 limit switch on left front bumper to identify hitting to foundation plate or other walls <BR>
 *      2 Color sensors pointing down one on left and another on right <BR>
 *      (to identify red / blue lines below skybridge for parking <BR>
 *      1 Hook servomotor to move base <BR>
 *
 *      Robot 1 : Chassis Motor : 5201 Series, 26:1 Ratio, 210 RPM Spur Gear Motor w/Encoder <BR>
 *      Encoder Countable Events Per Revolution (Output Shaft)	723.24 (Rises & Falls of Ch A & B) <BR>
 *
 *      Robot 2 : 5202 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder) <BR>
 *      Encoder Countable Events Per Revolution (Output Shaft)	537.6 (Rises & Falls of Ch A & B) <BR>
 *
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisMethods : moveHookServo()
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
 * @ChassisAutoMethods : frontleftChassisTouchSensorIsPressed()
 * @ChassisAutoMethods : moveHook_holdFoundation()
 * @ChassisAutoMethods : moveHook_Released()
 *
 */

public class Chassis {

    //Declare Chassis Motor and configurations
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Declare TouchSensor on front left and right of Chassis
    public TouchSensor frontleftChassisTouchSensor;
    public TouchSensor frontrightChassisTouchSensor;

    //Declare Color Sensors
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;

    // public Servo hook;
    //public Servo hook;
    public Servo lefthook;
    public Servo righthook;

    //Declare Chassis Configuration variables
    public double wheelRadius;
    public double robotRadius;
    public double target90degRotations;

    //Timer for timing out Arm motion incase targetPosition cannot be achieved
    ElapsedTime ChassisMotionTimeOut = new ElapsedTime();

    public boolean configureRobot = false;

    public double ChassisMotorEncoderCount = 723.24;

    //public double HOOK_HOLD = 0.87;
    //public double HOOK_RELEASED = 0.22;

    
    public double LEFT_HOOK_HOLD = 0.72;
    public double LEFT_HOOK_RELEASED = 0.0;

    public double RIGHT_HOOK_HOLD = 0.33;
    public double RIGHT_HOOK_RELEASED = 1.0;

    /**
     * Constructor of Chassis. <BR>
     * Sets hardwareMap on hub1 with 4 motors, 2 light sensors, 1 touch sensors <BR>
     * Configures Robot for size and mecanum wheel directions <BR>
     * Initialize Robot to right component modes. <BR>
     * @param hardwareMap HardwareMap to be setup on Hub1
     */
    public Chassis(HardwareMap hardwareMap) {
        //Map DCMotors from configuration
        frontLeft = hardwareMap.dcMotor.get("front_left_drive");
        frontRight = hardwareMap.dcMotor.get("front_right_drive");
        backLeft = hardwareMap.dcMotor.get("back_left_drive");
        backRight = hardwareMap.dcMotor.get("back_right_drive");

        //Map TouchSensor from configuration
        frontleftChassisTouchSensor = hardwareMap.touchSensor.get("left_touch_sensor");
        frontrightChassisTouchSensor = hardwareMap.touchSensor.get("right_touch_sensor");

        //Map ColorSensors from configuration
        leftColorSensor = hardwareMap.get(ColorSensor.class, "left_color");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "right_color");

        //Map Hook from configuration
        //hook = hardwareMap.servo.get("hook");
        lefthook = hardwareMap.servo.get("left_hook");
        righthook = hardwareMap.servo.get("right_hook");

        //Configure Robot to dimensions and modified for wheel type
        configureRobot();
    }

    /**
     * Configure Chassis for size and mecanum wheel directions
     */
    public void configureRobot(){
        wheelRadius = 1.965*1.1* 7/5; //100mm and applied correction factor of 7/5 for slippage
        robotRadius = 8.54*1.1*7/5; //Was 8.64 Radius = half of longest diagonal = 0.5*sqrt(sq(14.5)+sq(10.5).and applied correction factor of 7/5 for slippage
        //Set direction of motors wrt motor drive set up, so that wheels go forward +y power

        //target90degRotations = (Math.PI*robotRadius/2)/(2*Math.PI*wheelRadius);

        target90degRotations = 1.02; //1.0865; Original value


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
        //hook.setPosition(HOOK_RELEASED);
        lefthook.setPosition(LEFT_HOOK_RELEASED);
        righthook.setPosition(RIGHT_HOOK_RELEASED);
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder. <BR>
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for the motor <BR>
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
     * Function to set the behaviour of the motor on passing Zero power to the motor <BR>
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
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity),
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
    public void setRightColorSensorEnabled(boolean colorSensorEnabled){
             rightColorSensor.enableLed(colorSensorEnabled);
    }


    /**
     * Method to check for left Color Sensor crossing over Red Line
     * Used in Autonomous mode to stop below Red Skybridge after moving foundation to wall.
     * @return if Color Sensor is red
     */
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean leftColorSensorIsRed() {
        //Logic to detect Red R>400
        if (leftColorSensor.red()>400) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for right Color Sensor crossing over Red Line <BR>
     * Used in Autonomous mode to stop below Red Skybridge after moving blocks (Optional use)
     * @return if color sensor is red
     */
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean rightColorSensorIsRed() {
        //Logic to detect Red R>200 G<127 B<127
        if (rightColorSensor.red()>400){
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for right Color Sensor crossing over Blue Line <BR>
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
     * Method to check for right Color Sensor crossing over Blue Line <BR>
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
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Method to move chassis based on computed vector inputs for a set distance.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors
     *
     * @param distance +ve for forward, -ve for backward
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runFwdBackLeftRight(
            double distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = distance /Math.abs(distance);

        while (!callingOpMode.isStopRequested() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations)
                )
              ){
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
     * Method to move chassis based on computed vector inputs for a set max_stop_distance.
     * Till frontleftChassisTouchSensor is pressed.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors.
     * @param max_stop_distance in same unit of measure as wheelRadius
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runFwdTill_frontChassisTouchSensor_Pressed(
            double max_stop_distance,
            double power,
            LinearOpMode callingOpMode) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        while (!callingOpMode.isStopRequested() &&
                (!(frontleftChassisTouchSensor.isPressed() || frontrightChassisTouchSensor.isPressed()) &&
                        (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations)
                        )
                )
              ) {
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
     * Method to move chassis based on computed vector inputs for a set max_stop_distance.
     * Till team color is identified below Chassis.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors.
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runTill_ChassisLeftColorSensorIsRed(
            double max_stop_distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while ( !callingOpMode.isStopRequested() &&
                !leftColorSensorIsRed() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))
              ) {
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
     * Method to move chassis based on computed vector inputs for a set max_stop_distance.
     * Till team color is identified below Chassis.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors.
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runTill_ChassisLeftColorSensorIsBlue(
            double max_stop_distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!callingOpMode.isStopRequested() &&
                !leftColorSensorIsBlue() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))
              ) {
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
     * Method to move chassis based on computed vector inputs for a set max_stop_distance.
     * Till team color is identified below Chassis.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors.
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runTill_ChassisRightColorSensorIsRed(
            double max_stop_distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while ( !callingOpMode.isStopRequested() &&
                !rightColorSensorIsRed() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))
              ) {
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
     * Method to move chassis based on computed vector inputs for a set max_stop_distance.
     * Till team color is identified below Chassis.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors.
     * @param max_stop_distance Max distance to stop
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runTill_ChassisRightColorSensorIsBlue(
            double max_stop_distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = max_stop_distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = max_stop_distance /Math.abs(max_stop_distance);

        while (!callingOpMode.isStopRequested() &&
                !rightColorSensorIsBlue() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations))
              ) {
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
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void turnby90degree(
            int clockOrAntiClockwise,
            double power,
            LinearOpMode callingOpMode){
        resetChassis();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!callingOpMode.isStopRequested() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * target90degRotations))
              ) {
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
     * Method to identify when frontleftChassisTouchSensor is pressed.
     * frontleftChassisTouchSensor.getState() return true when not touched.
     *
     */
    public boolean frontChassisTouchSensorIsPressed(){
        if (frontleftChassisTouchSensor.isPressed() || frontrightChassisTouchSensor.isPressed()){
            return true;
        } else {
            return false;
        }
    }


    /**
     * Method to move hook  to hold on foundation
     */
    public void moveHook_holdFoundation(){

        //moveHookServo(HOOK_HOLD);
        moveHookServo(LEFT_HOOK_HOLD,RIGHT_HOOK_HOLD);
    }

    /**
     * Method to move hold to released default state
     */
    public void moveHook_Released(){
        //moveHookServo(HOOK_RELEASED);
        moveHookServo(LEFT_HOOK_RELEASED,RIGHT_HOOK_RELEASED);
    }


    /**
     * Mothod to move the hook to the set level
     * //@param hookLevel
     * LEFT_HOOK_HOLD = 1, LEFT_HOOK_RELEASED = 0.54
     * RIGHT_HOOK_HOLD = 0.46, RIGHT_HOOK_RELEASED = 0.93

     */
    public void moveHookServo(double lefthookLevel, double righthookLevel) {
    /*    if (hookLevel <= HOOK_RELEASED) {
            hookLevel = HOOK_RELEASED;
        }

        if (hookLevel >= HOOK_HOLD) {
            hookLevel = HOOK_HOLD;
        }
    */
        //Limit hook level to max values : LEFT_HOOK_HOLD = 0.73, LEFT_HOOK_RELEASED = 0.0
        if (lefthookLevel <= LEFT_HOOK_RELEASED) {
            lefthookLevel = LEFT_HOOK_RELEASED;
        }

        if (lefthookLevel >= LEFT_HOOK_HOLD) {
            lefthookLevel = LEFT_HOOK_HOLD;
        }

        //Limit hook level to max values : RIGHT_HOOK_HOLD = 0.27, RIGHT_HOOK_RELEASED = 1.0
        if (righthookLevel >= RIGHT_HOOK_RELEASED) {
            righthookLevel = RIGHT_HOOK_RELEASED;
        }

        if (righthookLevel <= RIGHT_HOOK_HOLD) {
            righthookLevel = RIGHT_HOOK_HOLD;
        }


        //hook.setPosition(hookLevel);
        lefthook.setPosition(lefthookLevel);
        righthook.setPosition(righthookLevel);
    }

}
