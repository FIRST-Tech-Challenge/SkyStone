package org.firstinspires.ftc.teamcode.competition;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {

    // Measurements and such kept as variables for ease of use
        // Ticks Per Rotation of an odometry wheel
    private static final double ODOM_TICKS_PER_ROTATION = 1440;
        // Radius of an odometry wheel in cm
    private static final double ODOM_WHEEL_RADIUS = 3.6;
        // Distance from left odometry wheel to the right odometry wheel in cm
    private static final double DIST_BETWEEN_WHEELS = 40.194;
        // Circumference of an odometry wheel in cm
    private static final double WHEEL_CIRCUM = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;
        // Number of ticks in a centimeter using dimensional analysis
    private static final double ODOM_TICKS_PER_CM = ODOM_TICKS_PER_ROTATION / WHEEL_CIRCUM;

    // Robot physical location
    public double x = 0;
    public double y = 0;
    public double theta = 0;

    // Hardware mapping
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    // Odometry hardware
    private DcMotor leftEncoder = null;
    private DcMotor rightEncoder = null;
    private DcMotor centerEncoder = null;

    // Rev Expansion Hub Data
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;

    // Odometry encoder positions
    public int leftEncoderPos = 0;
    public int centerEncoderPos = 0;
    public int rightEncoderPos = 0;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled = 0;
    public double rightOdomTraveled = 0;
    public double centerOdomTraveled = 0;


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
          // left front
        leftFront = hwMap.dcMotor.get("leftFront");
            // Motors don't have encoders on them because we're using odometry
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // When motors aren't receiving power, they will attempt to hold their position
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          // left rear
        leftRear = hwMap.dcMotor.get("leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          // right front
        rightFront = hwMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          // right rear
        rightRear = hwMap.dcMotor.get("rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry encoder setup
        leftEncoder = leftFront;
        rightEncoder = rightFront;  // These would need to be reversed anyway
        centerEncoder = rightRear;  //  so they are set to the motors which get reversed

        // Rev ExpansionHub Bulk Data
        expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        leftOdom = (ExpansionHubMotor) hwMap.dcMotor.get("leftFront");
        rightOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightFront");
        centerOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightRear");
    }

    /**
     * Update robot position using odometry
     */
    public void updatePosition() {
        // Get the circumference of the distance traveled by the wheel since the last update
        // Circumference multiplied by degrees the wheel has rotated
        double deltaLeftDist = getLeftTicks() / ODOM_TICKS_PER_CM;
        double deltaRightDist = getRightTicks() / ODOM_TICKS_PER_CM;
        double deltaCenterDist = getCenterTicks() / ODOM_TICKS_PER_CM;

        // Update real world distance traveled by the odometry wheels
        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        /* To get theta, we find the arc tangent of the difference between the two wheels'
            distance traveled divided by the distance between the two wheels */
        theta += Math.atan2((deltaLeftDist - deltaRightDist), DIST_BETWEEN_WHEELS);

        // Finds the unrotated point's position, then rotates it around the origin, then adjusts to robot position
        x += (deltaLeftDist + deltaRightDist) / 2.0 /* * Math.cos(theta) +
                ((deltaCenterDist) - Math.sin(theta))*/;
        y += (deltaCenterDist)/* * Math.cos(theta) +
                (((deltaLeftDist+deltaRightDist) / 2.0) - Math.sin(theta))*/;

        resetTicks();
    }

    public void resetTicks() {
        leftEncoderPos = bulkData.getMotorCurrentPosition(leftOdom);
        rightEncoderPos = bulkData.getMotorCurrentPosition(rightOdom);
        centerEncoderPos = bulkData.getMotorCurrentPosition(centerOdom);
    }

    public int getLeftTicks() { return bulkData.getMotorCurrentPosition(leftOdom) - leftEncoderPos; }

    public int getRightTicks() { return bulkData.getMotorCurrentPosition(rightOdom) - rightEncoderPos; }

    public int getCenterTicks() { return bulkData.getMotorCurrentPosition(centerOdom) - centerEncoderPos; }

    /**
     * Resets position of the robot to x=0, y=0, theta=0
     */
    public void resetPosition(){
        x = 0;
        y = 0;
        theta = 0;
        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        centerOdomTraveled = 0;
    }

    /**
     * Resets the encoder values to zero
     */
    public void resetEncoders(){
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Run mode needs to be reset because encoders and wheels are pointing to the same location
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}