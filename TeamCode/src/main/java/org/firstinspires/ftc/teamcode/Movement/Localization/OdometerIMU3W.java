package org.firstinspires.ftc.teamcode.Movement.Localization;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometerIMU3W extends Odometer{

    /* Top-Down View of The Bottom of a Robot

    /===================\
    |                   |
    |  A------X------B  |   A represents the left vertical dead-wheel
    |     ^         |   |   B represents the right vertical dead-wheel, and has to be in line with A
    |     H         |   |
    |           V-> |   |   C represents the horizontal dead-wheel, and can be placed anywhere
    |               C   |
    |                   |
    \===================/

    The above diagram is a top-down view of the bottom of the robot, meaning that when you are
    looking at the robot from the top, A is to the left, B is to the right, and C is to the back.

    X denotes the center of the robot
    H denotes the horizontalOffset variable, or distance from A to B divided by 2.
    V denotes the verticalOffset variable, or the vertical distance from C to X. Does not matter very much

    Odometer measurements can be in whatever units you want, as long as you use the same units for every constant
    */

    private double horizontalOffset = 10;
    private double verticalOffset = 10;

    private double rightVerticalDirection = 1;
    private double leftVerticalDirection = 1;
    private double horizontalDirection = 1;

    // Encoder Objects
    private DcMotor leftVerticalEncoder, rightVerticalEncoder, horizontalEncoder;
    //IMU
    private BNO055IMU imu;
    // Encoder Variables
    public double leftVertical, rightVertical, horizontal;
    private double lastLeftVertical, lastRightVertical, lastHorizontal;
    private double leftVerticalChange, rightVerticalChange, horizontalChange;
    private double ticksToDistance;
    // Math Variables
    private double headingChange, headingImu, lastHeadingImu;
    private double centerArc, turnRadius;
    private double horizontalAdjust, horizontalExtra;
    private double[] positionChangeVertical = {0, 0}; //Position change vector from vertical encoders
    private double[] positionChangeHorizontal = {0, 0}; //Position change vector from horizontal encoder
    private double[] totalRelativeMovement = {0, 0};
    private double[] totalPositionChange = {0, 0};

    public OdometerIMU3W(RobotHardware robotHardware){

        this.leftVerticalEncoder = robotHardware.leftBack;
        this.rightVerticalEncoder = robotHardware.rightFront;
        this.horizontalEncoder = robotHardware.rightBack;
        this.imu = robotHardware.imu;

    }

    @Override
    public void initialize(){

        leftVerticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lastLeftVertical = 0;
        lastRightVertical = 0;
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

        leftVertical = leftVerticalEncoder.getCurrentPosition() * ticksToDistance * leftVerticalDirection;
        rightVertical = rightVerticalEncoder.getCurrentPosition() * ticksToDistance * rightVerticalDirection;
        horizontal = horizontalEncoder.getCurrentPosition() * ticksToDistance * horizontalDirection;

        leftVerticalChange = leftVertical - lastLeftVertical;
        rightVerticalChange = rightVertical - lastRightVertical;
        horizontalChange = horizontal - lastHorizontal;

        headingImu = getImuHeading();

        headingChange = headingImu - lastHeadingImu;

        if (headingChange < -3){ // For example 355 to 2 degrees
            headingChange = 2*Math.PI + headingChange;
        }else if (headingChange > 3) { // For example 2 to 355 degrees
            headingChange = -2*Math.PI + headingChange;
        }

        headingRadians += headingChange;

        // Calculating the position-change-vector from two vertical encoders
        centerArc = (leftVerticalChange + rightVerticalChange) / 2;

        if(headingChange == 0) { // Robot has gone straight/not moved

            positionChangeVertical[0] = 0;
            positionChangeVertical[1] = centerArc;

        }else if(Math.abs(rightVerticalChange) < Math.abs(leftVerticalChange)){ //Left encoder is on inside of the turn

            turnRadius = centerArc/headingChange; //Always positive

            positionChangeVertical[0] = turnRadius - Math.cos(headingChange) * turnRadius;
            positionChangeVertical[1] = Math.sin(headingChange) * turnRadius;

        }else{ //Right encoder is on inside of the turn

            turnRadius = centerArc/-headingChange; //Always positive

            positionChangeVertical[0] = turnRadius - Math.cos(-headingChange) * turnRadius;
            positionChangeVertical[1] = Math.sin(-headingChange) * turnRadius;

        }

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

        lastLeftVertical = leftVertical;
        lastRightVertical = rightVertical;
        lastHorizontal = horizontal;

        heading = Math.toDegrees(headingRadians);

    }

    // Utility Methods
    public void setEncoderDirections(double rightVerticalDirection, double leftVerticalDirection, double horizontalDirection){

        this.rightVerticalDirection = rightVerticalDirection;
        this.leftVerticalDirection = leftVerticalDirection;
        this.horizontalDirection = horizontalDirection;

    }

    private double getImuHeading() {
        //May need to change axis unit to work with vertical hubs
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = (angles.firstAngle + 360) % 360;
        return Math.toRadians(heading);
    }

}
