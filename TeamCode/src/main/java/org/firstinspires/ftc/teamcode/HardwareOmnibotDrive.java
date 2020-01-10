package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

/**
 *Created by 12090 STEM Punk
 */
public class HardwareOmnibotDrive
{
    /* Public OpMode members. */
    public final static double MIN_SPIN_RATE = 0.1;
    public final static double MIN_DRIVE_RATE = 0.1;

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String REAR_LEFT_MOTOR = "RearLeft";
    public final static String REAR_RIGHT_MOTOR = "RearRight";
    public final static String LEFT_INTAKE = "LeftIntake";
    public final static String RIGHT_INTAKE = "RightIntake";
    public final static String EXTENDER = "Extender";
    // We need both hubs here because one has the motors, and the other has the
    // odometry encoders.
    public final static String HUB1 = "Expansion Hub 2";
    public final static String HUB2 = "Expansion Hub 3";


    // Hardware objects
    RevBulkData bulkDataHub1;
    //    ExpansionHubMotor leftIntakeBd, rightIntakeBd, lifterBd, extenderBd;
    boolean hub1Read = false;
    ExpansionHubEx expansionHub1;
    RevBulkData bulkDataHub2;
    boolean hub2Read = false;
    ExpansionHubEx expansionHub2;

    // These motors have the odometry encoders attached
    protected ExpansionHubMotor leftIntake = null;
    protected ExpansionHubMotor rightIntake = null;
    protected ExpansionHubMotor extender = null;

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
    public boolean defaultInputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;

    public static boolean encodersReset = false;
    public boolean forceReset = false;

    public double xAngle, yAngle, zAngle;
    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDrive(){
    }

    public int getLeftEncoderWheelPosition() {
        if(!hub1Read) {
            bulkDataHub1 = expansionHub1.getBulkInputData();
            hub1Read = true;
        }
        // This is to compensate for GF having a negative left.
        return -bulkDataHub1.getMotorCurrentPosition(leftIntake);
    }

    public int getRightEncoderWheelPosition() {
        if(!hub1Read) {
            bulkDataHub1 = expansionHub1.getBulkInputData();
            hub1Read = true;
        }
        return bulkDataHub1.getMotorCurrentPosition(rightIntake);
    }

    public int getStrafeEncoderWheelPosition() {
        if(!hub1Read) {
            bulkDataHub1 = expansionHub1.getBulkInputData();
            hub1Read = true;
        }
        return bulkDataHub1.getMotorCurrentPosition(extender);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Motor: Lifter, RightIntake, Extender, LeftIntake
        // Encoder: Lifter, LeftEncoder, CenterEncoder, RightEncoder
        expansionHub1 = hwMap.get(ExpansionHubEx.class, HUB1);
        // RearRight, RearLeft, FrontLeft, FrontRight
//        expansionHub2 = hwMap.get(ExpansionHubEx.class, HUB2);

        // Define and Initialize Motors
        frontLeft = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRight = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.dcMotor.get(REAR_LEFT_MOTOR);
        rearRight = hwMap.dcMotor.get(REAR_RIGHT_MOTOR);
        leftIntake = (ExpansionHubMotor) hwMap.dcMotor.get(LEFT_INTAKE);
        rightIntake = (ExpansionHubMotor) hwMap.dcMotor.get(RIGHT_INTAKE);
        extender = (ExpansionHubMotor) hwMap.dcMotor.get(EXTENDER);


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

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initIMU();
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        defaultInputShaping = inputShapingEnabled;
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

    public void resetReads() {
        imuRead = false;

        // Bulk data items
        hub1Read = false;
        hub2Read = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(Math.abs(power - frontLeftMotorPower) > 0.0005)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(Math.abs(power - rearLeftMotorPower) > 0.0005)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(Math.abs(power - frontRightMotorPower) > 0.0005)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(Math.abs(power - rearRightMotorPower) > 0.0005)
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
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        rearLeft.setPower(0.0);
        rearRight.setPower(0.0);
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset, boolean inputShaping) {
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(-45.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(-135.0 + gyroAngle);
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double joystickAngle = atan2(yPower, xPower);
        double newPower = driverInputShaping(joystickMagnitude, inputShaping);
        double newSpin = driverInputSpinShaping(spin, inputShaping);
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

    protected double driverInputShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut = 0.0;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_DRIVE_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_SPIN_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        // The Odometry Encoders
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // The Odometry Encoders
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*1.6;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*1.6;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*1.6;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*1.6;

        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        setFrontLeftMotorPower(tl_power_raw);
        setFrontRightMotorPower(tr_power_raw);
        setRearRightMotorPower(br_power_raw);
        setRearLeftMotorPower(bl_power_raw);
    }
}

