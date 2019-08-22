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
    private static final double ODOM_ROTATION_TICKS = 360;
    private static final double ODOM_WHEEL_RADIUS = 3;
    private static final double ODOM_WHEEL_DIST = 0; // TODO: Distance between odometry wheels

    // Robot physical location
    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public  double thetaCenter = 0;

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
        leftEncoder = hwMap.dcMotor.get("leftEncoder");
        rightEncoder = hwMap.dcMotor.get("rightEncoder");
        rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        centerEncoder = hwMap.dcMotor.get("centerEncoder");
        centerEncoder.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Update robot position using odometry
     */
    public void updatePosition() {
        // Get the circumference of the distance traveled by the wheel since the last update
        // Circumference multiplied by degrees the wheel has rotated
        double deltaLeftDist =  2.0 * Math.PI * ODOM_WHEEL_RADIUS * (getLeftTicks() / ODOM_ROTATION_TICKS);
        double deltaRightDist = 2.0 * Math.PI * ODOM_WHEEL_RADIUS * (getRightTicks() / ODOM_ROTATION_TICKS);
        // TODO: Create/find a method for using the third odometry wheel
        double deltaCenterDist = 2.0 * Math.PI * ODOM_WHEEL_RADIUS * (getCenterTicks() / ODOM_ROTATION_TICKS);
        x  += (((deltaLeftDist + deltaRightDist) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDist + deltaRightDist) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDist - deltaRightDist) / ODOM_WHEEL_DIST;
        thetaCenter += deltaCenterDist / ODOM_WHEEL_DIST;
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
}