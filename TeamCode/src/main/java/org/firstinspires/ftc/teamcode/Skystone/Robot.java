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
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection.VUFORIA_KEY;
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
    final double OUTTAKE_SLIDE_EXTENDED = .1;
    final double OUTTAKE_SLIDE_RETRACTED = .75;

    // Outtake Servo Positions
    final double CLAW_SERVO_CLAMPED = .51;
    final double CLAW_SERVO_RELEASED = .3;

    // Outtake Pivot Positions
    final double OUTTAKE_PIVOT_EXTENDED = .271;
    final double OUTTAKE_PIVOT_RETRACTED = .994;

    // Outtake Pusher Positions
    final double PUSHER_PUSHED = .7;
    final double PUSHER_RETRACTED = .452;

    double i = 1;

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
    private double wheelRadius = 2;
    private final double wheelCircumference = 4 * Math.PI;
    private final double encoderPerRevolution = 806.4;
    private final double l = 7;
    private final double w = 6.5;

    //PID (concept only)
    private double xMovement;
    private double yMovement;
    private double turnMovement;

    private double pathDistance;

    private double distanceToEnd;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone
        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        // Map intake motors
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        // Set direction of intake motors
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        // Map outtake motors
        outtakeSpool = hardwareMap.dcMotor.get("outtakeSpool");

        outtakeSpool.setDirection(DcMotor.Direction.REVERSE);

        outtakeExtender = hardwareMap.servo.get("outtakeExtender");
        clamp = hardwareMap.servo.get("clamp");
        clampPivot = hardwareMap.servo.get("clampPivot");
        intakePusher = hardwareMap.servo.get("intakePusher");
    }

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

    public void resetEncoders() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changeRunModeToUsingEncoder() {
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode) {
        fLeft.setMode(runMode);
        fRight.setMode(runMode);
        bLeft.setMode(runMode);
        bRight.setMode(runMode);
    }

    public void absoluteTurn (double targetHeadingRadians, double turnSpeed){
        targetHeadingRadians = angleWrap(targetHeadingRadians);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startHeading = angleWrap(anglePos);

        while (linearOpMode.opModeIsActive()){

            double currentAngle = anglePos;

            turnMovement = 0.77 * turnSpeed * (targetHeadingRadians - currentAngle) / (Math.abs(targetHeadingRadians - startHeading));

            telemetry.addLine("turnMovement" + turnMovement);
            telemetry.update();

            if (Math.abs(targetHeadingRadians - currentAngle) <= Math.toRadians(1)){

                telemetry.addLine("1 degree off, finish turn");
                telemetry.update();
                break;
            }
            applyMove(1);
        }

        brakeRobot();
        linearOpMode.sleep(100);
        turnMovement = 0;
    }

    //normal use method default 2 second kill time
    public void finalTurn(double targetHeading) {
        finalTurn(targetHeading, 2500);
    }

    public void finalTurn(double targetHeading, long timeInMilli) {
        targetHeading = Range.clip(targetHeading, -179, 179);
        long startTime = SystemClock.elapsedRealtime();
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        position = imu.getPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        int sign;
        if (targetHeading > startHeading) {
            sign = 1;
        } else {
            sign = -1;
        }
        if (startHeading == targetHeading) {
            return;
        }
        while (linearOpMode.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double scaleFactor = 0.9 * Math.abs((angles.firstAngle - startHeading) / (startHeading - targetHeading));
            double absolutePower = 1 - scaleFactor;
            if (absolutePower < 0.1) {
                brakeRobot();
                return;
            }
            double power = absolutePower * sign;
            if (scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)) {
                break;
            }
            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(-power);
            bRight.setPower(power);
        }
        brakeRobot();
        linearOpMode.sleep(100);
    }

    public void allWheelDrive(double fLpower, double fRpower, double bLpower, double bRpower) {
        fLeft.setPower(fLpower);
        fRight.setPower(fRpower);
        bLeft.setPower(bLpower);
        bRight.setPower(bRpower);
    }

    public void intake(boolean toggle) {
        if (toggle) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    public void resetMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

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

    public void moveRobot(double speed, int targetPostition) {
        //called by final move - bare bones move function
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        double newSpeed = speed;
        if (targetPostition < 0) {
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
        fLeft.setTargetPosition(targetPostition);
        fRight.setTargetPosition(targetPostition);
        bLeft.setTargetPosition(targetPostition);
        bRight.setTargetPosition(targetPostition);
        while (fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && linearOpMode.opModeIsActive()) {
        }
        brakeRobot();
        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMotorsBreakZeroBehavior() {
        //sets drive motors to brake mode
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakeRobot() {
        //brakes robot
        driveMotorsBreakZeroBehavior();
        fLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
        linearOpMode.sleep(250);
    }

    public void setBrakeModeDriveMotors() {
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

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


    public boolean followCurve(Vector<CurvePoint> allPoints, double followAngle) {
        Vector<CurvePoint> pathExtended = (Vector<CurvePoint>) allPoints.clone();

        pointWithIndex distanceAlongPath = distanceAlongPath(allPoints, robotPos);
        int currFollowIndex = distanceAlongPath.index + 1;

        CurvePoint followMe = getFollowPointPath(pathExtended, robotPos, allPoints.get(currFollowIndex).followDistance);

        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(allPoints.size() - 2), allPoints.get(allPoints.size() - 1), allPoints.get(allPoints.size() - 1).pointLength * 1.5));

        distanceToEnd = Math.hypot(distanceAlongPath.x - allPoints.get(allPoints.size() - 1).x, distanceAlongPath.y - allPoints.get(allPoints.size() - 1).y);

        if (distanceToEnd <= followMe.followDistance + 15 || Math.hypot(robotPos.x - allPoints.get(allPoints.size() - 1).x, robotPos.y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 15) {
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }

        goToPoint(followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followAngle);

        if ((distanceToEnd < 0.25)) {
            return false;
        }

        double decelerationScaleFactor = Range.clip(distanceToEnd/5,-1,1);
        applyMove(decelerationScaleFactor);
        return true;
    }

    public void moveFollowCurve(Vector<CurvePoint> points) {
        pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (linearOpMode.opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, Math.toRadians(0))) {
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
        turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
    }

    private void applyMove(double decelerationScaleFactor) {

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
        fLeft.setPower(fLeftPower * decelerationScaleFactor);
        fRight.setPower(fRightPower * decelerationScaleFactor);
        bLeft.setPower(bLeftPower * decelerationScaleFactor);
        bRight.setPower(bRightPower * decelerationScaleFactor);

        telemetry.addLine("deceleration: " + decelerationScaleFactor);
    }


    public void moveToPoint(double x, double y, double moveSpeed, double turnSpeed, double optimalAngle) {

        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (linearOpMode.opModeIsActive()) {
            double xPos = robotPos.x;
            double yPos = robotPos.y;
            double anglePos = this.anglePos;

            double distanceToTarget = Math.hypot(x - xPos, y - yPos);

            if (distanceToTarget < 2) {
                brakeRobot();
                break;
            }

            double absoluteAngleToTarget = Math.atan2(y - yPos, x - xPos);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
            double relativeTurnAngle = relativeAngleToPoint + optimalAngle;

            double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

            xMovement = xPower * moveSpeed;
            yMovement = yPower * moveSpeed;
            turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

            double decelerationScaleFactor = Range.clip(distanceToTarget/8,-1,1);

            applyMove(decelerationScaleFactor);
        }
    }

    public String detectVuforia(String VUFORIA_KEY) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters paramaters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(paramaters);

        OpenGLMatrix lastLocation = new OpenGLMatrix();
        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable skyStoneTarget = targetsSkyStone.get(0);
        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        final float mmPerInch = 25.4f;

        targetsSkyStone.activate();
        int num = 0;
        long startTime = SystemClock.elapsedRealtime();

        while (linearOpMode.opModeIsActive()) {
            telemetry.addLine("in loop");
            telemetry.update();
            if (((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).isVisible()) {
                telemetry.addLine("Visible");
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                VectorF translation = lastLocation.getTranslation();

                if (translation.get(0) / mmPerInch > 7.5) {
                    telemetry.addData("Position: ", "left");
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    return "left";

                } else if (translation.get(0) / mmPerInch < 7.5){
                    telemetry.addData("Position: ", "Center");
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    telemetry.update();
                    return "center";
                } else {
                    telemetry.addData("Position: ", "Right");
                    return "Right";
                }
            }
            if (SystemClock.elapsedRealtime() - startTime > 5000) {
                telemetry.addLine("No detection");
                telemetry.update();
                return "none";
            }
        }

        return "none";
    }
    private VuforiaLocalizer initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        return ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod(VuforiaLocalizer vuforia) {
        final String TFOD_MODEL_ASSET = "Skystone.tflite";
        final String LABEL_FIRST_ELEMENT = "Stone";
        final String LABEL_SECOND_ELEMENT = "Skystone";
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.50;
        TFObjectDetector tfod;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        return tfod;
    }


    public int detectTensorflow(){
        /**
         * if it sees something/object detected : get the confidence
         * if confidence is greater than 0.7 : find its position and return that
         * if confidence is less than 0.7 : get the position it thinks, go through the code again, if is the same, then return that
         */
        VuforiaLocalizer vuforia = initVuforia();
        TFObjectDetector tfod;
        tfod = initTfod(vuforia);
        tfod.activate();
        long startTime = SystemClock.elapsedRealtime();

        // 2 is right, 1 is center, 0 is left
        ArrayList<Integer> retVals = new ArrayList<>();

        // scan for 5 seconds
        while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime()-startTime<5000){

            // get all the detections
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            // if there is a detection run the logic
            if (updatedRecognitions != null && updatedRecognitions.size()>0) {

                // sorts based on confidence levels
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        return (int)(recognition.getConfidence()-t1.getConfidence());
                    }
                });

                // iterate through each recognition
                for (int i = 0; i < updatedRecognitions.size(); i++){

                    // value is the center of the detection
                    float value = (updatedRecognitions.get(i).getTop()+updatedRecognitions.get(i).getBottom())/2;

                    // if the confidence is greater than 0.9, then return that
                    if ((double)updatedRecognitions.get(i).getConfidence() > 0.9){
                        if (value < 600){
                            return 2;
                        } else if (value < 800){
                            retVals.add(1);
                            return 1;
                        } else {
                            return 0;
                        }
                    }
                    // if the confidence is greater than 0.5, add it to the arraylist
                    else if ((double)updatedRecognitions.get(i).getConfidence() > 0.5) {
                        if (value < 600){
                            retVals.add(2);
                        } else if (value < 800) {
                            retVals.add(1);
                        } else {
                            retVals.add(0);
                        }
                    }
                }
            }
        }

        // find the average of everything in the arraylist
        double retVal = 0;
        for (int i = 0; i < retVals.size(); i++){
            retVal += retVals.get(i);
        }
        retVal /= retVals.size();

        telemetry.addLine("retVal" + retVal);
        telemetry.update();

        // return rounded int average
        return (int)Math.round(retVal);
    }

    public void goToVuforia(){

        final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

         while (linearOpMode.opModeIsActive()){

            changeRunModeToUsingEncoder();
            Robot robot = new Robot(this.hardwareMap, this.telemetry, this.linearOpMode);
            // start odometry
            Position2D position2D = new Position2D(robot);
            position2D.startOdometry();

            robot.resetEncoders();

            robot.changeRunModeToUsingEncoder();
            telemetry.addLine("Got into runopmode");

            robot.moveToPoint(11.5 ,0,0.4,1,Math.toRadians(0));
            telemetry.addLine("done with move");
            telemetry.update();

            telemetry.addLine("go to point");
            int position = 0;
            int vuforiaPosition = robot.detectTensorflow();
            robot.intakeRight.setPower(1);

            robot.intakeLeft.setPower(1);

            if (vuforiaPosition == 2) {
                telemetry.addLine("left");
                robot.moveToPoint(39, 9, 0.55, 0.5, Math.toRadians(0));
            } else if (vuforiaPosition == 0){
                telemetry.addLine("right");
                robot.moveToPoint(39, -9, 0.55, 0.5, Math.toRadians(0));
            } else {
                telemetry.addLine("center");
                robot.moveToPoint(39, 0, 0.55, 0.5, Math.toRadians(0));
            }
            telemetry.addLine("Done with detect");
            telemetry.update();
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);

            telemetry.addLine("Done with move");
            telemetry.update();

        }
    }

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

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
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

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
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

    public double getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
    }

    public double getEncoderPerRevolution() {
        return encoderPerRevolution;
    }

    public double getL() {
        return l;
    }

    public double getW() {
        return w;
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

    public double getPathDistance() {
        return pathDistance;
    }

    public void setPathDistance(double pathDistance) {
        this.pathDistance = pathDistance;
    }

    public double getDistanceToEnd() {
        return distanceToEnd;
    }

    public void setDistanceToEnd(double distanceToEnd) {
        this.distanceToEnd = distanceToEnd;
    }
}