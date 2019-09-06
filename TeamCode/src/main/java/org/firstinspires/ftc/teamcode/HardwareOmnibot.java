package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

/**
 *Created by Ethan
 */
public class HardwareOmnibot
{
    /* Public OpMode members. */
    // Shooter speeds
    public final static double MAX_SPIN_RATE = 0.7;
    public final static double MIN_SPIN_RATE = 0.05;
    public final static double MIN_DRIVE_RATE = 0.05;
    public final static double MAX_LIFT_RATE = 1.0;
    public final static double MIN_EXTEND_RATE = 0.25;
    public final static double MAX_EXTEND_RATE = 1.0;
    public final static double MIN_ROTATE_RATE = 0.25;
    public final static double MAX_ROTATE_RATE = 0.40;
    public final static int MAX_FAST_EXTENSION = 6000;
    public final static int MAX_EXTENSION = 7150;
    public final static int MIN_FAST_EXTENSION = 1000;
    public final static int MIN_EXTENSION = 100;

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String BACK_LEFT_MOTOR = "RearLeft";
    public final static String BACK_RIGHT_MOTOR = "RearRight";
    public final static String LIFT_MOTOR = "LanderMotor";
    public final static String ROTATOR1_MOTOR = "Rotator1";
    public final static String ROTATOR2_MOTOR = "Rotator2";
    public final static String EXTENDER_MOTOR = "Extender";
    public final static String LEFT_COLLECTOR = "CollectorLeft";
    public final static String RIGHT_COLLECTOR = "CollectorRight";
    public final static String LEFT_COLLECTOR_SENSOR = "ColorLeft";
    public final static String RIGHT_COLLECTOR_SENSOR = "ColorRight";


    protected DcMotor leftMotorFore = null;
    protected DcMotor rightMotorFore = null;
    protected DcMotor leftMotorRear = null;
    protected DcMotor rightMotorRear = null;
    protected BNO055IMU imu = null;
    protected DcMotor lifter = null;
    protected DcMotor rotator1 = null;
    protected DcMotor rotator2 = null;
    protected DcMotor extender = null;
    protected CRServo leftCollector = null;
    protected CRServo rightCollector = null;
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    protected ColorSensor sensorColorRight;
    protected DistanceSensor sensorDistanceRight;
    protected ColorSensor sensorColorLeft;
    protected DistanceSensor sensorDistanceLeft;

    private static final int encoderClicksPerSecond = 2800;
    protected double leftForeMotorPower = 0.0;
    protected double leftRearMotorPower = 0.0;
    protected double rightForeMotorPower = 0.0;
    protected double rightRearMotorPower = 0.0;
    private double liftMotorPower = 0.0;
    private double rotatorMotorPower = 0.0;
    private double extenderMotorPower = 0.0;
    private double leftCollectorPower = 0.0;
    private double rightCollectorPower = 0.0;
    private boolean inputShaping = true;

    // Thread to run the LEDs
    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    /* local OpMode members. */
    private HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibot(){
    }

    public void initGroundEffects()
    {
        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);
        leds.setLength(60);
        colors = new ArrayList<Integer>();
        colors.add(0, 0x0);
        colors.add(1, 0x0);
        halfAndHalf = new DSPatternHalfAndHalf(leds);
        halfAndHalf.setPatternColors(colors);
        ftcTimer = new DSPatternFtcTimer(leds);
        robotDisplay = halfAndHalf;
    }

    public void updateTimerGroundEffects() {
        robotDisplay.update();
    }

    public void startTimerGroundEffects() {
        robotDisplay = ftcTimer;
        robotDisplay.update();
    }

    public void updateElementColors(int leftColor, int rightColor) {
        colors.set(0, leftColor);
        colors.set(1, rightColor);
        robotDisplay.setPatternColors(colors);
        robotDisplay.update();
    }

    public void stopGroundEffects() {
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        inputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    /*public void calibrateGyro()
    {
        // Calibrate Gyro Code
        gyro.calibrate();
        while(gyro.isCalibrating()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
    }

    public void resetGyro()
    {
        // Reset Gyro Code
        //gyro.resetZAxisIntegrator();
    }*/

    public double readIMU()
    {
        // Read IMU Code
        //double heading = (double)gyro.getHeading();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = (double)angles.firstAngle;
        //heading = abs(heading - 360.0);
        return heading;
    }

    public void setLeftCollectorPower(double power)
    {
        if(power != leftCollectorPower)
        {
            leftCollectorPower = power;
            leftCollector.setPower(power);
        }
    }

    public void setRightCollectorPower(double power)
    {
        if(power != rightCollectorPower)
        {
            rightCollectorPower = power;
            rightCollector.setPower(power);
        }
    }

    public void setLeftForeMotorPower(double power)
    {
        if(power != leftForeMotorPower)
        {
            leftForeMotorPower = power;
            leftMotorFore.setPower(power);
        }
    }

    public void setLeftRearMotorPower(double power)
    {
        if(power != leftRearMotorPower)
        {
            leftRearMotorPower = power;
            leftMotorRear.setPower(power);
        }
    }

    public void setRightForeMotorPower(double power)
    {
        if(power != rightForeMotorPower)
        {
            rightForeMotorPower = power;
            rightMotorFore.setPower(power);
        }
    }

    public void setRightRearMotorPower(double power)
    {
        if(power != rightRearMotorPower)
        {
            rightRearMotorPower = power;
            rightMotorRear.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setLeftForeMotorPower(power);
        setRightForeMotorPower(power);
        setRightRearMotorPower(power);
        setLeftRearMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    public void setLiftMotorPower(double power){

        power /= max(power, 1.0);

        if (power < MIN_DRIVE_RATE && power > -MIN_DRIVE_RATE) {
            power = 0;
        }

        power *= MAX_LIFT_RATE;

        if(power != liftMotorPower) {
            liftMotorPower = power;
            lifter.setPower(power);
        }
    }

    public void setExtenderMotorPower (double power, boolean override) {

        power /= max(power, 1.0);
        int extension = extender.getCurrentPosition();

        if (abs(power) < MIN_EXTEND_RATE){
            power = 0;
        }
        power *= MAX_EXTEND_RATE;

        // This is to allow us to retract when the robot starts extended
        if(override) {
            extension = (MAX_EXTENSION + MIN_EXTENSION) / 2;
            power *= 0.25;
        }

        if(power > 0) {
            if(extension >= MAX_EXTENSION) {
                power = 0;
            } else if (extension >= MAX_FAST_EXTENSION) {
                power *= 0.25;
            }
        } else if (power < 0) {
            if(extension <= MIN_EXTENSION) {
                power = 0;
            } else if (extension <= MIN_FAST_EXTENSION) {
                power *= 0.25;
            }
        }

        if(power != extenderMotorPower) {
            extenderMotorPower = power;
            extender.setPower(power);
        }
    }

    public void startLifterUp() {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(8500);
        setLiftMotorPower(1.0);
    }

    public void startLifterDown() {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(0);
        setLiftMotorPower(1.0);
    }

    public void stopLifter() {
        setLiftMotorPower(0.0);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRotatorMotorPower (double power) {

        power /= max(power, 1.0);

        if (abs(power) < MIN_ROTATE_RATE){
            power = 0;
        }

        power *= MAX_ROTATE_RATE;

        if(power != rotatorMotorPower) {
            rotatorMotorPower = power;
            rotator1.setPower(power);
            rotator2.setPower(power);
        }
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void newDrive(double xPower, double yPower, double spin, double angleOffset) {
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(-45.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(-135.0 + gyroAngle);
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double joystickAngle = atan2(yPower, xPower);
        double newPower = driverInputShaping(joystickMagnitude);
        double newSpin = driverInputSpinShaping(spin);
        double newXPower = newPower * cos(joystickAngle);
        double newYPower = newPower * sin(joystickAngle);

        double LFpower = newXPower * cos(leftFrontAngle) + newYPower * sin(leftFrontAngle) + newSpin;
        double LRpower = newXPower * cos(leftRearAngle) + newYPower * sin(leftRearAngle) + newSpin;
        double RFpower = newXPower * cos(rightFrontAngle) + newYPower * sin(rightFrontAngle) + newSpin;
        double RRpower = newXPower * cos(rightRearAngle) + newYPower * sin(rightRearAngle) + newSpin;

        double maxPower = max(1.0, max(max(abs(LFpower), abs(LRpower)),
                max(abs(RFpower), abs(RRpower))));

        if(maxPower > 1.0) {
            LFpower /= maxPower;
            RFpower /= maxPower;
            RFpower /= maxPower;
            RRpower /= maxPower;
        }

        leftMotorFore.setPower(LFpower);
        leftMotorRear.setPower(LRpower);
        rightMotorFore.setPower(RFpower);
        rightMotorRear.setPower(RRpower);
    }

    protected double driverInputShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_DRIVE_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                } else {
                    valueOut = -MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }
    protected double driverInputSpinShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_SPIN_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn - MIN_SPIN_RATE;
                } else {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn + MIN_SPIN_RATE;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }
    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset)
    {
        // Read Gyro Angle Here
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(315.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(225.0 + gyroAngle);

        double expYPower = driverInputShaping(yPower);
        double expXPower = driverInputShaping(xPower);
        double expSpinPower = driverInputShaping(spin);

        double LFpower = 2*(expXPower * cos(leftFrontAngle) + expYPower * sin(leftFrontAngle))/sqrt(2) + expSpinPower;
        double LRpower = 2*(expXPower * cos(leftRearAngle) + expYPower * sin(leftRearAngle))/sqrt(2) + expSpinPower;
        double RFpower = 2*(expXPower * cos(rightFrontAngle) + expYPower * sin(rightFrontAngle))/sqrt(2) + expSpinPower;
        double RRpower = 2*(expXPower * cos(rightRearAngle) + expYPower * sin(rightRearAngle))/sqrt(2) + expSpinPower;

        double maxPower = max(1.0, max(max(abs(LFpower), abs(LRpower)),
                max(abs(RFpower), abs(RRpower))));

        if(maxPower > 1.0) {
            LFpower /= maxPower;
            RFpower /= maxPower;
            RFpower /= maxPower;
            RRpower /= maxPower;
        }

        setLeftForeMotorPower(LFpower);
        setLeftRearMotorPower(LRpower);
        setRightForeMotorPower(RFpower);
        setRightRearMotorPower(RRpower);
    }

    public void resetDriveEncoders()
    {
        int sleepTime = 0;
        int encoderCount = leftMotorFore.getCurrentPosition();

        rightMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = leftMotorFore.getCurrentPosition();
        }

        leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the stop mode
        leftMotorFore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFore = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        rightMotorFore  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        leftMotorRear = hwMap.dcMotor.get(BACK_LEFT_MOTOR);
        rightMotorRear = hwMap.dcMotor.get(BACK_RIGHT_MOTOR);
        lifter = hwMap.dcMotor.get(LIFT_MOTOR);
        leftCollector = hwMap.crservo.get(LEFT_COLLECTOR);
        rightCollector = hwMap.crservo.get(RIGHT_COLLECTOR);
        extender = hwMap.dcMotor.get(EXTENDER_MOTOR);
        rotator1 = hwMap.dcMotor.get(ROTATOR1_MOTOR);
        rotator2 = hwMap.dcMotor.get(ROTATOR2_MOTOR);
        sensorColorRight = hwMap.get(ColorSensor.class, RIGHT_COLLECTOR_SENSOR);
        sensorColorLeft = hwMap.get(ColorSensor.class, LEFT_COLLECTOR_SENSOR);
        sensorDistanceRight = hwMap.get(DistanceSensor.class, RIGHT_COLLECTOR_SENSOR);
        sensorDistanceLeft = hwMap.get(DistanceSensor.class, LEFT_COLLECTOR_SENSOR);
        // Set up the LEDs. Change this to your configured name.
        leds = hwMap.get(DotStarBridgedLED.class, "leds");


        leftMotorFore.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFore.setDirection(DcMotor.Direction.FORWARD);
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD);
        rightMotorRear.setDirection(DcMotor.Direction.FORWARD);
        lifter.setDirection(DcMotor.Direction.REVERSE);
        leftCollector.setDirection(CRServo.Direction.REVERSE);
        rightCollector.setDirection(CRServo.Direction.FORWARD);
        extender.setDirection(DcMotor.Direction.REVERSE);
        rotator1.setDirection(DcMotor.Direction.REVERSE);
        rotator2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        setAllDriveZero();
        setLiftMotorPower(0.0);

        resetDriveEncoders();

        initIMU();
        // Don't init ground effects here because this is also used for autonomous
        initGroundEffects();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

