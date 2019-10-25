package org.firstinspires.ftc.teamcode.base_classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class Robot {
    //TODO: add getters and setters

    OpMode opMode;

    //Declares motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    // Direction of motion
    public double forwardPower = 0;
    public double leftPower = 0;

    public double getForwardPower() {
        return forwardPower;
    }

    public void setForwardPower(double forwardPower) {
        this.forwardPower = forwardPower;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public Robot() {

    }

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initializes telemetry, motors, servos
     */
    public void init() {
        //initializes the motors
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "fl");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "fr");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "bl");
        backRight = opMode.hardwareMap.get(DcMotor.class, "br");

        //sets direction of the motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //FROM ftc samples
        //sets zero power behavior of the motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    /**
     * Turns all the wheels in the same direction to move uniformly forwards or backwards in a straight line
     *
     * @param power percentage of full power given to each motor
     */
    public void driveStraight(double power) {
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        this.setForwardPower(power);
        this.setLeftPower(0);
    }

    /**
     * Strafes left
     *
     * @param power percentage of full power given to each motor
     */
    public void strafeLeft(double power) {
        //@TODO: figure out if there should be a minimum power at 0, since if power < 0, robot will move right
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        this.setForwardPower(0);
        this.setLeftPower(power);
    }

    /**
     * Strafes right
     *
     * @param power percentage of full power given to each motor
     */
    public void strafeRight(double power) {
        strafeLeft(-power);
    }

    /**
     * Stops the wheels
     */
    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        this.setForwardPower(0);
        this.setLeftPower(0);
    }

    /**
     * Rotates clockwise
     *
     * @param power percentage of full power given to each motor
     */
    public void rotateClockwise(double power) {
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        this.setForwardPower(0);
        this.setLeftPower(0);
    }

    /**
     * Rotates counterclockwise
     *
     * @param power percentage of full power given to each motor
     */
    public void rotateCounter(double power) {
        rotateClockwise(-power);
    }
}
