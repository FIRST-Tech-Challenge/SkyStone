package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.CurvePoint;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import static java.lang.Thread.sleep;
//import static org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.lineCircleIntersection;

public class Robot {
    //Drive Motors
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    // Intake Motors
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Outtake Motors
    private DcMotor outtakeSpool;

    // Outtake Servos
    private Servo outtakeExtender;
    private Servo clamp;
    private Servo clampPivot;
    private Servo intakePusher;

    // Outtake Slide Positions
    public final double OUTTAKE_SLIDE_EXTENDED = .25;
    public final double OUTTAKE_SLIDE_RETRACTED = .75;

    // Outtake Clamp Positions
    public final double CLAW_SERVO_CLAMPED = .45;
    public final double CLAW_SERVO_RELEASED = .335;

    // Outtake Pivot Positions
    public final double OUTTAKE_PIVOT_EXTENDED = .271;
    public final double OUTTAKE_PIVOT_RETRACTED = .994;
    public final double OUTTAKE_PIVOT_90 = 0.6325;

    // Outtake Pusher Positions
    public final double PUSHER_PUSHED = .75;
    public final double PUSHER_RETRACTED = .475;

    //robots position
    private Point robotPos = new Point();
    private double anglePos;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    //Inherited classes from Op Mode
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;

    //dimensions
    private final double encoderPerRevolution = 806.4;
    private double xMovement;
    private double yMovement;
    private double turnMovement;

    /**
     * robot constructor, does the hardwareMaps
     * @param hardwareMap
     * @param telemetry
     * @param linearOpMode
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone

        //Map drive motors
        fLeft = getDcMotor("fLeft");
        if (fLeft != null){
            fLeft.setDirection(DcMotor.Direction.FORWARD);

        }
        fRight = getDcMotor("fRight");
        if (fRight != null){
            fRight.setDirection(DcMotor.Direction.REVERSE);

        }
        bLeft = getDcMotor("bLeft");
        if (bLeft != null){
            bLeft.setDirection(DcMotor.Direction.FORWARD);

        }
        bRight = getDcMotor("bRight");
        if (bRight != null){
            bRight.setDirection(DcMotor.Direction.REVERSE);

        }

        intakeLeft = getDcMotor("intakeLeft");
        if (intakeLeft != null){
            intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        intakeRight = getDcMotor("intakeRight");
        if (intakeRight != null){
            intakeRight.setDirection(DcMotor.Direction.REVERSE);

        }

        outtakeSpool = getDcMotor("outtakeSpool");
        if (outtakeSpool != null){
            outtakeSpool.setDirection(DcMotor.Direction.REVERSE);
        }

        outtakeExtender = hardwareMap.servo.get("outtakeExtender");
        clamp = hardwareMap.servo.get("clamp");
        clampPivot = hardwareMap.servo.get("clampPivot");
        intakePusher = hardwareMap.servo.get("intakePusher");
    }

    private DcMotor getDcMotor(String name){
        try {
            return hardwareMap.dcMotor.get(name);

        } catch (IllegalArgumentException exception){
            return null;
        }
    }

    private Servo getServo(String name){
        try {
            return hardwareMap.servo.get(name);

        } catch (IllegalArgumentException exception){
            return null;
        }
    }

    /**
     * call this when you want to use the imu in a program
     */
    public void intializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    }

    /**
     * resets all the encoders back to 0
     */
    public void resetEncoders() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * make all the motors run using encoder
     * do NOT use this if you want to set your own powers (e.x deceleration)
     */
    public void changeRunModeToUsingEncoder() {
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * sets drive motors mode to whatever
     * @param runMode what to set all the motors to
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        fLeft.setMode(runMode);
        fRight.setMode(runMode);
        bLeft.setMode(runMode);
        bRight.setMode(runMode);
    }

//    //normal use method default 2 second kill time
//    public void finalTurn(double targetHeading, double turnSpeed) {
//        finalTurn(targetHeading, turnSpeed, 2500);
//    }

    /**
     * crappy finalTurn, random deceleration
     * turns to the absolute heading using odometry
     * @param targetHeadingRadians
     */
    public void finalTurn(double targetHeadingRadians){

        targetHeadingRadians = angleWrap(targetHeadingRadians);

        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = 1;

        double direction = 0;

        while (linearOpMode.opModeIsActive()){

            // find which direction to turn
            if (targetHeadingRadians > anglePos){
                direction = 1;
            } else if (targetHeadingRadians < anglePos){
                direction = -1;
            }

            // lol deceleration
            if (Math.abs(targetHeadingRadians - anglePos) < Math.toRadians(0.15)){
                break;
            } else if (Math.abs(targetHeadingRadians - anglePos) < Math.toRadians(5)){
                power = 0.15;
            } else if (Math.abs(targetHeadingRadians - anglePos) < Math.toRadians(30)){
                power = 0.2;
            } else if (Math.abs(targetHeadingRadians - anglePos) < Math.toRadians(45)){
                power = 0.4;
            }

            // sets the powers
            fLeft.setPower(power * direction);
            bLeft.setPower(power * direction);
            fRight.setPower(-power * direction);
            bRight.setPower(-power * direction);
        }

        // upon "break"
        brakeRobot();

//        targetHeading = Range.clip(targetHeading, -179, 179);
//
//        long startTime = SystemClock.elapsedRealtime();
//
//        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        position = imu.getPosition();
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        double startHeading = angles.firstAngle;
//        double maxAngle = startHeading - targetHeading;
//        maxAngle = Math.abs(maxAngle);
//
//        int sign = 0;
//        if(targetHeading > startHeading){
//            sign = 1;
//        }else{
//            sign = -1;
//        }
//        if(maxAngle == 0){
//            return;
//        }
//        while(linearOpMode.opModeIsActive()){
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
//            double scaleFactor = currentDeltatAngle / maxAngle;
//            double absolutePower = 1-scaleFactor;
//
//            if(absolutePower< 0.01){
//                absolutePower = 0.01;
//            }
//            double power = absolutePower * sign;
//            if(scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)){
//                break;
//            }
//            fLeft.setPower(-power * turnSpeed);
//            fRight.setPower(power * turnSpeed);
//            bLeft.setPower(-power * turnSpeed);
//            bRight.setPower(power * turnSpeed);
//        }
//        brakeRobot();
//        linearOpMode.sleep(100);
//        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * sets all the motor speeds independently
     * @param fLpower
     * @param fRpower
     * @param bLpower
     * @param bRpower
     */
    public void allWheelDrive(double fLpower, double fRpower, double bLpower, double bRpower) {
        fLeft.setPower(fLpower);
        fRight.setPower(fRpower);
        bLeft.setPower(bLpower);
        bRight.setPower(bRpower);
    }

    /**
     * allows to toggle intake
     * @param toggle if true, intake, if false, stop intake
     */
    public void intake(boolean toggle) {
        if (toggle) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    /**
     * idk
     * @param motor the motor you want to reset
     */
    public void resetMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * move straight forwards or backwards
     * @param speed speed to move, >0, <1
     * @param targetDistance, target distance, -infinity to infinity
     */
    public void finalMove(double speed, double targetDistance) {
        //move robot function
        //to move backwards make targetDistance negative
        double rotations = 0;
        if (targetDistance > 0) {
            rotations = targetDistance / 0.0168;
        } else {
            rotations = targetDistance / 0.0156;
        }
        moveRobot(speed, (int) (rotations));
        brakeRobot();
        linearOpMode.sleep(100);
    }

    /**
     * bare bones move function using encoders
     * @param speed speed to move
     * @param targetPosition target distance
     */
    public void moveRobot(double speed, int targetPosition) {
        //called by final move - bare bones move function
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        double newSpeed = speed;
        if (targetPosition < 0) {
            newSpeed = newSpeed * -1;
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        } else {
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        }
        fLeft.setTargetPosition(targetPosition);
        fRight.setTargetPosition(targetPosition);
        bLeft.setTargetPosition(targetPosition);
        bRight.setTargetPosition(targetPosition);
        while (fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && linearOpMode.opModeIsActive()) {
        }
        brakeRobot();
        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * idk
     */
    public void driveMotorsBreakZeroBehavior() {
        //sets drive motors to brake mode
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * brakes all the motors
     */
    public void brakeRobot() {
        //brakes robot
        driveMotorsBreakZeroBehavior();
        fLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
        linearOpMode.sleep(250);
    }

    /**
     * idk
     */
    public void setBrakeModeDriveMotors() {
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * moves along spline, hella bad bc scaling
     * @param data
     */
    public void splineMove(double[][] data) {
        resetEncoders();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        long startTime = SystemClock.elapsedRealtime();
        double refLeftSpeed, refRightSpeed;
        double refLeftDistance, refRightDistance;
        double refHeading;
        double leftPower, rightPower;
        double leftDistance, rightDistance;
        double heading;
        int inc;
        int i;
        double encoderToInches = 0.156;  //515 encoders = 8 inches
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        /*double maxSpeed = 0;
        for (i=0; i<data.length; i++){
            if (maxSpeed < data[i][0]){
                maxSpeed = data[i][0];
            }
            if (maxSpeed < data[i][1]){
                maxSpeed = data[i][1];
            }
        }
        */
        while (linearOpMode.opModeIsActive()) {
            double dt = SystemClock.elapsedRealtime() - startTime; //in milli
            dt = dt / 1000; //in seconds
            if (dt < data[data.length - 1][2]) {                //find increment at any time to do interpolation
                inc = -1;
                for (i = 0; i < data.length - 2; i++) {
                    if (data[i][2] <= dt && dt < data[i + 1][2]) {
                        inc = i;
                        break;
                    }
                }
                if (inc < 0) {
                    brakeRobot();
                    break;
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // find the left and right speed by interpolation from data file
                refLeftSpeed = ((data[inc + 1][0] - data[inc][0]) / (data[inc + 1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][0];
                refRightSpeed = ((data[inc + 1][1] - data[inc][1]) / (data[inc + 1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][1];
                // find the left and right distance by interpolation from data file
                refLeftDistance = ((data[inc + 1][4] - data[inc][4]) / (data[inc + 1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
                refRightDistance = ((data[inc + 1][5] - data[inc][5]) / (data[inc + 1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
                // find the heading by interpolation from data file
                refHeading = ((data[inc + 1][6] - data[inc][6]) / (data[inc + 1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][6] + startHeading;
                // find the left and right encoder values and convert them to distance traveled in inches
                leftDistance = fLeft.getCurrentPosition() * encoderToInches;
                rightDistance = fRight.getCurrentPosition() * encoderToInches;
                // find the heading of robot
                heading = angles.firstAngle;
                // find power
                leftPower = refLeftSpeed;
                rightPower = refRightSpeed;
                // set power
                fLeft.setPower(leftPower);
                bLeft.setPower(leftPower);
                fRight.setPower(rightPower);
                bRight.setPower(rightPower);
            } else {
                brakeRobot();
                break;
            }
        }
    }



    public boolean followCurve(double[][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance) {

        PathPoints path = new PathPoints(points,followDistance);

        Vector<CurvePoint> allPoints = path.targetPoints;

        // for angle locking
        Point secondToLastPoint = new Point(points[points.length-2][0],points[points.length-2][1]);
        Point lastPoint = new Point(points[points.length-1][0],points[points.length-1][1]);
        double lastAngle = Math.atan2(lastPoint.y - secondToLastPoint.y, lastPoint.x - secondToLastPoint.x);

        Vector<CurvePoint> pathExtended = (Vector<CurvePoint>) allPoints.clone();

        pointWithIndex distanceAlongPath = distanceAlongPath(allPoints, robotPos);
        int currFollowIndex = distanceAlongPath.index + 1;

        CurvePoint followMe = getFollowPointPath(pathExtended, robotPos, allPoints.get(currFollowIndex).followDistance);

        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(allPoints.size() - 2), allPoints.get(allPoints.size() - 1), allPoints.get(allPoints.size() - 1).pointLength * 1.5));

        double distanceToEnd = Math.hypot(distanceAlongPath.x - allPoints.get(allPoints.size() - 1).x, distanceAlongPath.y - allPoints.get(allPoints.size() - 1).y);

        if (distanceToEnd <= followMe.followDistance + 15 || Math.hypot(robotPos.x - allPoints.get(allPoints.size() - 1).x, robotPos.y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 15) {
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }

        // check if finished pp
        if ((distanceToEnd < 0.5)) {
            return false;
        }

        // get the movements
        goToPoint(followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followAngle);

        // angle lock
        if (distanceToEnd < angleLockDistance){
            if (Math.abs(anglePos-angleLockRadians) < Math.toRadians(2)){
                followAngle = 0;
            } else {
                turnMovement = (angleLockRadians - angleWrap(anglePos + 2*Math.PI))/1.25;
            }
        }

        // get the motor powers
        applyMove();
        return true;
    }

    public void moveFollowCurve(double [][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance) {
        //pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (linearOpMode.opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, followAngle, followDistance,angleLockRadians, angleLockDistance)) {
                brakeRobot();
                return;
            }
        }
    }

    public void moveFollowCurveWithExtend(double [][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance, Point whereYouExtend) {

        long outtakeExecutionTime = 0;
        long currentTime;
        boolean hasExtended = false;
        boolean isExtend = false;
        boolean isMoving = true;
        while (linearOpMode.opModeIsActive()) {
            currentTime = SystemClock.elapsedRealtime();

            // if followCurve returns false then it is ready to stop
            // else, it moves
            if (isMoving && !followCurve(points, followAngle, followDistance,angleLockRadians, angleLockDistance)) {
                brakeRobot();
                isMoving = false;
            }

            if (Math.hypot(whereYouExtend.x - robotPos.x, whereYouExtend.y - robotPos.y) < 40 && !hasExtended){
                hasExtended = true;
                isExtend = true;
                outtakeExecutionTime = SystemClock.elapsedRealtime();
                intakePusher.setPosition(PUSHER_PUSHED); // Push block all the way to clamp
            }

            if (currentTime - outtakeExecutionTime >= 500 && isExtend) {
                intakePusher.setPosition(PUSHER_RETRACTED);
            }
            if (currentTime - outtakeExecutionTime >= 850 && isExtend) {
                clamp.setPosition(CLAW_SERVO_CLAMPED);
            }
            if(currentTime-outtakeExecutionTime >= 1000 && isExtend){
                outtakeExtender.setPosition(OUTTAKE_SLIDE_EXTENDED);
            }

            if(currentTime-outtakeExecutionTime >= 2000 && isExtend){
                clampPivot.setPosition(OUTTAKE_PIVOT_90);
            }

            if(currentTime-outtakeExecutionTime >=2250){
                isExtend = false;
            }

            if(!isMoving && !isExtend){
                return;
            }
        }
    }

    public void moveFollowCurveWithRetract(double [][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance, Point whereYouRetract) {

        long outtakeExecutionTime = 0;
        long currentTime;
        boolean isRetract = false;
        boolean isMoving = true;
        boolean hasRetracted = false;
        while (linearOpMode.opModeIsActive()) {
            currentTime = SystemClock.elapsedRealtime();

            // if followCurve returns false then it is ready to stop
            // else, it moves
            if (isMoving && !followCurve(points, followAngle, followDistance,angleLockRadians, angleLockDistance)) {
                brakeRobot();
                isMoving = false;
            }

            if (Math.hypot(whereYouRetract.x - robotPos.x, whereYouRetract.y - robotPos.y) < 40 && !hasRetracted){
                outtakeExecutionTime = SystemClock.elapsedRealtime();
                intakePusher.setPosition(PUSHER_PUSHED); // Push block all the way to clamp
                hasRetracted = true;
                telemetry.addLine("in range");
                isRetract = true;
            }

            if(currentTime-outtakeExecutionTime >= 450 && isRetract){
                clampPivot.setPosition(OUTTAKE_PIVOT_RETRACTED);
                telemetry.addLine("pivot retracted");
            }
            if(currentTime-outtakeExecutionTime >= 1150 && isRetract){
                outtakeExtender.setPosition(OUTTAKE_SLIDE_RETRACTED);
                clamp.setPosition(CLAW_SERVO_RELEASED);
                intakePusher.setPosition(PUSHER_RETRACTED);
                telemetry.addLine("slide retracted");
                isRetract = false;
            }

            if(!isMoving && !isRetract){
                telemetry.addLine("done with movefollowcurve");
                return;
            }
        }
    }


    public boolean followCurve(double[][] points, double followAngle, double followDistance) {

        PathPoints path = new PathPoints(points,followDistance);

        Vector<CurvePoint> allPoints = path.targetPoints;

        Vector<CurvePoint> pathExtended = (Vector<CurvePoint>) allPoints.clone();

        pointWithIndex distanceAlongPath = distanceAlongPath(allPoints, robotPos);
        int currFollowIndex = distanceAlongPath.index + 1;

        CurvePoint followMe = getFollowPointPath(pathExtended, robotPos, allPoints.get(currFollowIndex).followDistance);

        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(allPoints.size() - 2), allPoints.get(allPoints.size() - 1), allPoints.get(allPoints.size() - 1).pointLength * 1.5));

        double distanceToEnd = Math.hypot(distanceAlongPath.x - allPoints.get(allPoints.size() - 1).x, distanceAlongPath.y - allPoints.get(allPoints.size() - 1).y);

        if (distanceToEnd <= followMe.followDistance + 15 || Math.hypot(robotPos.x - allPoints.get(allPoints.size() - 1).x, robotPos.y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 15) {
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }

        double decelerationScaleFactor = Range.clip(distanceToEnd/12,-1,1);

        goToPoint(followMe.x, followMe.y, followMe.moveSpeed * decelerationScaleFactor, followMe.turnSpeed * decelerationScaleFactor, followAngle);
        if ((distanceToEnd < 0.5)) {
            return false;
        }

        applyMove();
        return true;
    }

    public void moveFollowCurve(double [][] points, double followAngle, double followDistance) {
        //pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (linearOpMode.opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, followAngle, followDistance)) {
                brakeRobot();
                return;
            }
        }
    }

    public static class pointWithIndex {
        private double x;
        private double y;
        private int index;

        public pointWithIndex(double xPos, double yPos, int index) {
            this.x = xPos;
            this.y = yPos;
            this.index = index;
        }
    }

    public static pointWithIndex distanceAlongPath(Vector<CurvePoint> pathPoints, Point robot) {
        double closestDistance = Integer.MAX_VALUE;

        int closestDistanceIndex = 0;

        Point distanceAlongLine = new Point();

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint firstPoint = pathPoints.get(i);
            CurvePoint secondPoint = pathPoints.get(i + 1);

            Point currentDistanceAlongLine = distanceAlongLine(firstPoint, secondPoint, robot);

            double distanceToClip = Math.hypot(robot.x - currentDistanceAlongLine.x, robot.y - currentDistanceAlongLine.y);

            if (distanceToClip < closestDistance) {
                closestDistance = distanceToClip;
                closestDistanceIndex = i;
                distanceAlongLine = currentDistanceAlongLine;
            }
        }
        //return the three things
        return new pointWithIndex(distanceAlongLine.x, distanceAlongLine.y, closestDistanceIndex);//now return the closestDistanceIndex
    }

    public static Point distanceAlongLine(CurvePoint line1, CurvePoint line2, Point robot) {
        if (line1.x == line2.x) {
            line1.x = line2.x + 0.01;
        }
        if (line1.y == line2.y) {
            line1.y = line2.y + 0.01;
        }

        //calculate the slope of the line
        double m1 = (line2.y - line1.y) / (line2.x - line1.x);
        //calculate the slope perpendicular to this line
        double m2 = (line1.x - line2.x) / (line2.y - line1.y);

        //clip the robot's position to be on the line
        double xAlongLine = ((-m2 * robot.x) + robot.y + (m1 * line1.x) - line1.y) / (m1 - m2);
        double yAlongLine = (m1 * (xAlongLine - line1.x)) + line1.y;
        return new Point(xAlongLine, yAlongLine);
    }

    public CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {
        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y, secondPoint.x - firstPoint.x);
        double lineLength = Math.hypot(secondPoint.x - firstPoint.x, secondPoint.y - firstPoint.y);
        //extend the line by 1.5 pointLengths
        double extendedLineLength = lineLength + distance;

        CurvePoint extended = new CurvePoint(secondPoint);
        extended.x = Math.cos(lineAngle) * extendedLineLength + firstPoint.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + firstPoint.y;
        return extended;
    }

    private CurvePoint getFollowPointPath(Vector<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            Vector<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = Double.MAX_VALUE;

            for (Point intersectionPoint : intersections) {
                double angle = Math.atan2(intersectionPoint.y - robotPos.y, intersectionPoint.x - robotPos.x);
                double deltaAngle = Math.abs(MathFunctions.angleWrap(angle - anglePos));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(intersectionPoint);
                }
            }
        }
        return followMe;
    }

    public void goToPoint(double x, double y, double moveSpeed, double turnSpeed, double optimalAngle) {

        double distanceToTarget = Math.hypot(x - robotPos.x, y - robotPos.y);
        double absoluteAngleToTarget = Math.atan2(y - robotPos.y, x - robotPos.x);

        double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToPoint + optimalAngle;

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        xMovement = xPower * moveSpeed;
        yMovement = yPower * moveSpeed;
        turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(270), -1, 1) * turnSpeed;
    }

    private void applyMove() {

        // convert movements to motor powers
        double fLeftPower = (yMovement * 1.414 + turnMovement + xMovement);
        double fRightPower = (-yMovement * 1.414 - turnMovement + xMovement);
        double bLeftPower = (-yMovement * 1.414 + turnMovement + xMovement);
        double bRightPower = (yMovement * 1.414 - turnMovement + xMovement);

        //scale all powers to below 1
        double maxPower = Math.abs(fLeftPower);
        if (Math.abs(bLeftPower) > maxPower) {
            maxPower = Math.abs(bLeftPower);
        }
        if (Math.abs(bRightPower) > maxPower) {
            maxPower = Math.abs(bRightPower);
        }
        if (Math.abs(fRightPower) > maxPower) {
            maxPower = Math.abs(fRightPower);
        }
        double scaleDownAmount = 1.0;
        if (maxPower > 1.0) {
            scaleDownAmount = 1.0 / maxPower;
        }
        fLeftPower *= scaleDownAmount;
        fRightPower *= scaleDownAmount;
        bLeftPower *= scaleDownAmount;
        bRightPower *= scaleDownAmount;

        // apply movement with decelerationScaleFactor
        fLeft.setPower(fLeftPower);
        fRight.setPower(fRightPower);
        bLeft.setPower(bLeftPower);
        bRight.setPower(bRightPower);
    }

    public void moveToPoint(double x, double y, double moveSpeed, double turnSpeed, double optimalAngle) {

        // find the total distance from the start point to the end point
        double totalDistanceToTarget = Math.hypot(x - robotPos.x, y - robotPos.y);

        double totalTimeSeconds = totalDistanceToTarget/20;

        // so deceleration works
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // start a timer
        long startTime = SystemClock.elapsedRealtime();

        // keep on running this
        while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime() - startTime < totalTimeSeconds * 1000) {

            // store your current position in variables
            double xPos = robotPos.x;
            double yPos = robotPos.y;
            double anglePos = this.anglePos;

            // find your current distance to target
            double distanceToTarget = Math.hypot(x - xPos, y - yPos);

            // only way to break the loop, if the distance to target is less than 1
            if (distanceToTarget < 1) {
                break;
            }

            // find the absolute angle to target
            double absoluteAngleToTarget = Math.atan2(y - yPos, x - xPos);
            // find the relative angle of the target to the robot
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
            // x distance for the robot to its target
            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            // y distance for the robot to its target
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
            // adds optimal angle
            double relativeTurnAngle = relativeAngleToPoint + optimalAngle;

            // converting the relativeX and relativeY to xPower and yPower
            double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

            // find the deceleration
            double decelerationScaleFactor = Range.clip(2 * Math.sqrt(Math.pow(totalDistanceToTarget,2) - Math.pow(totalDistanceToTarget - distanceToTarget,2)) / totalDistanceToTarget,-1,1);

            // get everything into x, y, and turn movements for applyMove
            // the robot can be viewed as something that moves on a coordinate plane
            // that moves in a x and y direction but also has a heading, where it is pointing
            xMovement = xPower * moveSpeed * decelerationScaleFactor;
            yMovement = yPower * moveSpeed * decelerationScaleFactor;
            turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30),
                    -1, 1) * turnSpeed * decelerationScaleFactor;

            applyMove();
        }
        brakeRobot();
    }

//    public int detectVuforia(String VUFORIA_KEY) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        VuforiaLocalizer.Parameters paramaters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
//        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(paramaters);
//
//        OpenGLMatrix lastLocation = new OpenGLMatrix();
//        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
//        VuforiaTrackable skyStoneTarget = targetsSkyStone.get(0);
//        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
//        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        final float mmPerInch = 25.4f;
//
//        targetsSkyStone.activate();
//        int num = 0;
//        long startTime = SystemClock.elapsedRealtime();
//
//        while (linearOpMode.opModeIsActive()) {
//            telemetry.addLine("in loop");
//            telemetry.update();
//            if (((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).isVisible()) {
//                telemetry.addLine("Visible");
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//                VectorF translation = lastLocation.getTranslation();
//
//                if (translation.get(0) / mmPerInch > 7.5) {
//                    telemetry.addData("Position: ", "left");
//                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//                    return 2;
//
//                } else if (translation.get(0) / mmPerInch < 7.5){
//                    telemetry.addData("Position: ", "Center");
//                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//                    telemetry.update();
//                    return 1;
//                } else {
//                    telemetry.addData("Position: ", "Right");
//                    return 0;
//                }
//            }
//            if (SystemClock.elapsedRealtime() - startTime > 5000) {
//                telemetry.addLine("No detection");
//                telemetry.update();
//                return 1;
//            }
//        }
//
//        return 1;
//    }
//    private VuforiaLocalizer initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        //  Instantiate the Vuforia engine
//        return ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }
//
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private TFObjectDetector initTfod(VuforiaLocalizer vuforia) {
//        final String TFOD_MODEL_ASSET = "Skystone.tflite";
//        final String LABEL_FIRST_ELEMENT = "Stone";
//        final String LABEL_SECOND_ELEMENT = "Skystone";
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minimumConfidence = 0.50;
//        TFObjectDetector tfod;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//        return tfod;
//    }
//
//
//    public int detectTensorflow(){
//
//        VuforiaLocalizer vuforia = initVuforia();
//        TFObjectDetector tfod;
//        tfod = initTfod(vuforia);
//        tfod.activate();
//        long startTime = SystemClock.elapsedRealtime();
//
//        // 2 is right, 1 is center, 0 is left
//        ArrayList<Integer> recognitions = new ArrayList<>();
//
//        // scan for 5 seconds
//        while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime() - startTime < 1000) {
//            telemetry.addLine("got into loop");
//            telemetry.update();
//            // get all the detections
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            telemetry.addLine("made List");
//            telemetry.update();
//
//            // if there is a detection run the logic
//            if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
//                telemetry.addLine("got into if");
//                telemetry.update();
//                // sorts based on confidence levels
//                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
//                    @Override
//                    public int compare(Recognition recognition, Recognition t1) {
//                        return (int) (recognition.getConfidence() - t1.getConfidence());
//                    }
//                });
//
//                // iterate through each recognition
//                for (int i = 0; i < updatedRecognitions.size(); i++) {
//                    telemetry.addLine("Got into second for loop");
//                    telemetry.update();
//                    // value is the center of the detection
//                    float value = (updatedRecognitions.get(i).getTop() + updatedRecognitions.get(i).getBottom()) / 2;
//
//                    // if the confidence is greater than 0.9, then return that
//                    if ((double) updatedRecognitions.get(i).getConfidence() > 0.9) {
//                        telemetry.addLine("0.9 confidence");
//                        telemetry.update();
//                        if (value < 600) {
//                            return 2;
//                        } else if (value < 800) {
//                            return 1;
//                        } else {
//                            return 0;
//                        }
//                    }
//                    // if the confidence is greater than 0.5, add it to the arraylist
//                    else if ((double) updatedRecognitions.get(i).getConfidence() > 0.5) {
//                        telemetry.addLine("0.5 confidence");
//                        telemetry.update();
//                        if (value < 600) {
//                            recognitions.add(2);
//                        } else if (value < 800) {
//                            recognitions.add(1);
//                            return 1;
//                        } else {
//                            recognitions.add(0);
//                        }
//                    }
//                }
//            }
//            telemetry.addLine("Bottom of loop");
//            telemetry.update();
//        }
//
//        double sum = 0;
//        for (int i = 0; i < recognitions.size(); i++){
//            sum += recognitions.get(i);
//        }
//
//        double retVal = sum / recognitions.size();
//
//        return (int)Math.round(retVal);
//    }
//
//    private void incrementBy(HashMap<Integer, Integer> recognitions, int key, int val){
//        Integer currentVal = recognitions.get(key);
//        if (currentVal == null){
//            recognitions.put(key, val);
//        } else {
//            currentVal += val;
//            recognitions.put(key, currentVal);
//        }
//    }



    public DcMotor getfLeft() {
        return fLeft;
    }

    public void setfLeft(DcMotor fLeft) {
        this.fLeft = fLeft;
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public void setfRight(DcMotor fRight) {
        this.fRight = fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public void setbLeft(DcMotor bLeft) {
        this.bLeft = bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }

    public void setbRight(DcMotor bRight) {
        this.bRight = bRight;
    }

    public DcMotor getIntakeLeft() {
        return intakeLeft;
    }

    public void setIntakeLeft(DcMotor intakeLeft) {
        this.intakeLeft = intakeLeft;
    }

    public DcMotor getIntakeRight() {
        return intakeRight;
    }

    public void setIntakeRight(DcMotor intakeRight) {
        this.intakeRight = intakeRight;
    }

    public DcMotor getOuttakeSpool() {
        return outtakeSpool;
    }

    public void setOuttakeSpool(DcMotor outtakeSpool) {
        this.outtakeSpool = outtakeSpool;
    }

    public Servo getOuttakeExtender() {
        return outtakeExtender;
    }

    public void setOuttakeExtender(Servo outtakeExtender) {
        this.outtakeExtender = outtakeExtender;
    }

    public Servo getClamp() {
        return clamp;
    }

    public void setClamp(Servo clamp) {
        this.clamp = clamp;
    }

    public Servo getClampPivot() {
        return clampPivot;
    }

    public void setClampPivot(Servo clampPivot) {
        this.clampPivot = clampPivot;
    }

    public Servo getIntakePusher() {
        return intakePusher;
    }

    public void setIntakePusher(Servo intakePusher) {
        this.intakePusher = intakePusher;
    }

    public double getOUTTAKE_SLIDE_EXTENDED() {
        return OUTTAKE_SLIDE_EXTENDED;
    }

    public double getOUTTAKE_SLIDE_RETRACTED() {
        return OUTTAKE_SLIDE_RETRACTED;
    }

    public double getCLAW_SERVO_CLAMPED() {
        return CLAW_SERVO_CLAMPED;
    }

    public double getCLAW_SERVO_RELEASED() {
        return CLAW_SERVO_RELEASED;
    }

    public double getOUTTAKE_PIVOT_EXTENDED() {
        return OUTTAKE_PIVOT_EXTENDED;
    }

    public double getOUTTAKE_PIVOT_RETRACTED() {
        return OUTTAKE_PIVOT_RETRACTED;
    }

    public Point getRobotPos() {
        return robotPos;
    }

    public void setRobotPos(Point robotPos) {
        this.robotPos = robotPos;
    }

    public double getAnglePos() {
        return anglePos;
    }

    public void setAnglePos(double anglePos) {
        this.anglePos = anglePos;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public Orientation getAngles() {
        return angles;
    }

    public void setAngles(Orientation angles) {
        this.angles = angles;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public LinearOpMode getLinearOpMode() {
        return linearOpMode;
    }

    public void setLinearOpMode(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public double getEncoderPerRevolution() {
        return encoderPerRevolution;
    }

    public double getxMovement() {
        return xMovement;
    }

    public void setxMovement(double xMovement) {
        this.xMovement = xMovement;
    }

    public double getyMovement() {
        return yMovement;
    }

    public void setyMovement(double yMovement) {
        this.yMovement = yMovement;
    }

    public double getTurnMovement() {
        return turnMovement;
    }

    public void setTurnMovement(double turnMovement) {
        this.turnMovement = turnMovement;
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }
}