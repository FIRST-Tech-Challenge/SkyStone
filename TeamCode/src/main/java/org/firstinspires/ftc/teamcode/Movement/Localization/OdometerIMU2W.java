package org.firstinspires.ftc.teamcode.Movement.Localization;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometerIMU2W extends Odometer{

    /* Top-Down View of The Bottom of a Robot

    /===================\
    |                   |
    |  A------X-----O   |   A represents the vertical dead-wheel, and can be placed anywhere
    |     ^         |   |
    |     H         |   |   B represents the horizontal dead-wheel, and can be placed anywhere
    |           V-> |   |
    |               B   |
    |                   |
    \===================/

    The above diagram is a top-down view of the bottom of the robot, meaning that when you are
    looking at the robot from the top, A is to the left, O is to the right, and B is to the back.

    X denotes the center of the robot
    H denotes the horizontalOffset variable, or distance from A to X.
    V denotes the verticalOffset variable, or the distance from O to B. Does not matter very much

    Odometer measurements can be in whatever units you want, as long as you use the same units for every constant
    */

    private double horizontalOffset = 6.24;
    private double verticalOffset = -16.1;

    //Directions without using intake
    private double verticalDirection = -1;
    private double horizontalDirection = 1;

    // Encoder Objects
    private DcMotor verticalEncoder, horizontalEncoder;
    //IMU
    private BNO055IMU imu;
    // Encoder Variables
    public  double vertical, horizontal;
    private double lastVertical, lastHorizontal;
    private double verticalChange, horizontalChange;
    private double ticksToDistance;
    // Math Variables
    private double headingChange, headingImu, lastHeadingImu;
    private double verticallAdjust, verticalExtra;
    private double horizontalAdjust, horizontalExtra;
    private double[] positionChangeVertical = {0, 0}; //Position change vector from vertical encoders
    private double[] positionChangeHorizontal = {0, 0}; //Position change vector from horizontal encoder
    private double[] totalRelativeMovement = {0, 0};
    private double[] totalPositionChange = {0, 0};

    public OdometerIMU2W(){

        this.verticalEncoder = RobotHardware.intakeLeft;
        this.horizontalEncoder = RobotHardware.intakeRight;
        this.imu = RobotHardware.imu;

    }

    @Override
    public void initialize(){

        verticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lastVertical = 0;
        lastHorizontal = 0;

        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(Params);

        ticksToDistance = wheelRadius*2*Math.PI/ticksPerRevolution*gearRatio;
    }

    @Override
    public void update(){

        vertical = verticalEncoder.getCurrentPosition() * ticksToDistance * verticalDirection;
        horizontal = horizontalEncoder.getCurrentPosition() * ticksToDistance * horizontalDirection;

        verticalChange = vertical - lastVertical;
        horizontalChange = horizontal - lastHorizontal;

        headingImu = getImuHeading();

        headingChange = headingImu - lastHeadingImu;

        if (headingChange < -3){ // For example 355 to 2 degrees
            headingChange = 2*Math.PI + headingChange;
        }else if (headingChange > 3) { // For example 2 to 355 degrees
            headingChange = -2*Math.PI + headingChange;
        }

        headingRadians += headingChange;

        // Calculating the position-change-vector from vertical encoder
        verticallAdjust = verticalOffset * headingChange;
        verticalExtra = verticalChange - verticallAdjust;

        positionChangeVertical[1] = Math.cos(headingChange) * verticalExtra;
        positionChangeVertical[0] = Math.sin(headingChange) * verticalExtra;

        //Calculating the position-change-vector from horizontal encoder
        horizontalAdjust = horizontalOffset * headingChange;
        horizontalExtra = horizontalChange - horizontalAdjust;

        positionChangeHorizontal[0] = Math.cos(headingChange) * horizontalExtra;
        positionChangeHorizontal[1] = Math.sin(headingChange) * horizontalExtra;

        //Add the two vectors together
        totalRelativeMovement[0] = positionChangeVertical[0] + positionChangeHorizontal[0];
        totalRelativeMovement[1] = positionChangeVertical[1] + positionChangeHorizontal[1];

        //Rotate the vector
        totalPositionChange[0] = totalRelativeMovement[0] * Math.cos(lastHeadingRadians) - totalRelativeMovement[1] * Math.sin(lastHeadingRadians);
        totalPositionChange[1] = totalRelativeMovement[0] * Math.sin(lastHeadingRadians) + totalRelativeMovement[1] * Math.cos(lastHeadingRadians);

        x = lastX + totalPositionChange[0];
        y = lastY + totalPositionChange[1];

        lastX = x;
        lastY = y;
        lastHeadingRadians = headingRadians;
        lastHeadingImu = headingImu;

        lastVertical = vertical;
        lastHorizontal = horizontal;

        heading = Math.toDegrees(headingRadians);

    }

    // Utility Methods
    public void setEncoderDirections(double verticalDirection, double horizontalDirection){

        this.verticalDirection = verticalDirection;
        this.horizontalDirection = horizontalDirection;

    }

    private double getImuHeading() {
        //May need to change axis unit to work with vertical hubs
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = (angles.firstAngle + 360) % 360;
        return Math.toRadians(heading);
    }

}
