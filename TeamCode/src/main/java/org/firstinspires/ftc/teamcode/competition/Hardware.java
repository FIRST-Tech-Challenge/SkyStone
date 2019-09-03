package org.firstinspires.ftc.teamcode.competition;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {

    // Measurements and such kept as variables for ease of use
    private static final double ODOM_ROTATION_TICKS = 1440; // Pules Per Minute of the encoders
    private static final double ODOM_WHEEL_RADIUS = 3.6;
    private static final double ODOM_WHEEL_DIST = 39.37;
    private static final double DIST_FROM_CENTER_OF_TURN = 0;

    // Robot physical location
    public double x = 0;
    public double y = 0;
    public double theta = 0;

    // Hardware mapping
    private HardwareMap hwMap;

    // Gyro
    BNO055IMU imu;

    // Drive train
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    // Odometry hardware
    private DcMotor leftEncoder = null;
    private DcMotor rightEncoder = null;
    private DcMotor centerEncoder = null;

    // Odometry encoder positions
    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;


    /**
     * Simple constructor to set hardware mapping to null
     */
    public Hardware() { hwMap = null; }

    /**
     * Initialization of hardware
     * @param mapping hardware map passed into class
     */
    public void init(HardwareMap mapping){
        hwMap = mapping;

        // Drive train motor setup
        leftFront = hwMap.dcMotor.get("leftFront");
        leftRear = hwMap.dcMotor.get("leftRear");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear = hwMap.dcMotor.get("rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Odometry encoder setup
        leftEncoder = leftFront;
        rightEncoder = rightFront;  // These would need to be reversed anyway
        centerEncoder = rightRear;  //  so they are set to the motors which get reversed
    }

    /**
     * Update robot position using odometry
     */
    public void updatePosition() {
        double wheelCircum = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;

        // Get the circumference of the distance traveled by the wheel since the last update
        // Circumference multiplied by degrees the wheel has rotated
        double deltaLeftDist =  wheelCircum * (getLeftTicks() / ODOM_ROTATION_TICKS);
        double deltaRightDist = wheelCircum * (getRightTicks() / ODOM_ROTATION_TICKS);
        double deltaCenterDist = wheelCircum * (getCenterTicks() / ODOM_ROTATION_TICKS);

        // Average of the change in the wheel multiplied by the cos/sin of the theta to account for rotation
        double deltaTheta = (deltaLeftDist - deltaRightDist) / ODOM_WHEEL_DIST;
        theta += deltaTheta;
        // TODO: Make theta have a clipped range (0 to 2pi) use division so that it finds in between

        /* Ensures that when spinning the X and Y values don't change
        Adding the movement measured by front encoders and the movement measured by the back
        encoder then subtracting the measurement from the turning on the back encoder
         */
        x  += ((deltaLeftDist+deltaRightDist) / 2.0) * Math.sin(theta) +
                (deltaCenterDist-deltaTheta)* DIST_FROM_CENTER_OF_TURN * Math.cos(theta);
        y  += ((deltaLeftDist+deltaRightDist) / 2.0) * Math.cos(theta) +
                (deltaCenterDist-deltaTheta)* DIST_FROM_CENTER_OF_TURN * Math.sin(theta);

        resetTicks();
    }

    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }

    public void resetLeftTicks() {
        leftEncoderPos = leftEncoder.getCurrentPosition();
    }

    public int getLeftTicks() {
        return leftEncoder.getCurrentPosition() - leftEncoderPos;
    }

    public void resetRightTicks() {
        rightEncoderPos = rightEncoder.getCurrentPosition();
    }

    public int getRightTicks() {
        return rightEncoder.getCurrentPosition() - rightEncoderPos;
    }

    public void resetCenterTicks() {
        centerEncoderPos = centerEncoder.getCurrentPosition();
    }

    public int getCenterTicks() {
        return centerEncoder.getCurrentPosition() - centerEncoderPos;
    }

    /**
     * Resets position of the robot to x=0, y=0, theta=0
     */
    public void resetPosition(){
        x = 0;
        y = 0;
        theta = 0;
    }

    /**
     * Resets the encoder values to zero
     */
    public void resetEncoders(){
        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;
    }
}