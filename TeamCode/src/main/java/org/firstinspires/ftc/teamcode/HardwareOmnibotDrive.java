package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

/**
 *Created by Ethan
 */
public class HardwareOmnibotDrive
{
    /* Public OpMode members. */
    public final static double MAX_SPIN_RATE = 0.7;
    public final static double MIN_SPIN_RATE = 0.05;
    public final static double MIN_DRIVE_RATE = 0.05;

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String REAR_LEFT_MOTOR = "RearLeft";
    public final static String REAR_RIGHT_MOTOR = "RearRight";


    // Hardware objects
    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor rearLeft = null;
    protected DcMotor rearRight = null;
    protected BNO055IMU imu = null;

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    private boolean inputShaping = true;

    public double xAngle, yAngle, zAngle;
    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDrive(){
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

    public double readIMU()
    {
        // Read IMU Code
        //double heading = (double)gyro.getHeading();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = abs(heading - 360.0);
        zAngle = (double)angles.firstAngle;
        yAngle = (double)angles.secondAngle;
        xAngle = (double)angles.thirdAngle;
        return (double)angles.firstAngle;
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(power != frontLeftMotorPower)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(power != rearLeftMotorPower)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(power != frontRightMotorPower)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(power != rearRightMotorPower)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset) {
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

        setFrontLeftMotorPower(LFpower);
        setFrontRightMotorPower(RFpower);
        setRearRightMotorPower(RRpower);
        setRearLeftMotorPower(LRpower);
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

    public void resetDriveEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = frontLeft.getCurrentPosition();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRight  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.dcMotor.get(REAR_LEFT_MOTOR);
        rearRight = hwMap.dcMotor.get(REAR_RIGHT_MOTOR);


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetDriveEncoders();

        initIMU();
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

