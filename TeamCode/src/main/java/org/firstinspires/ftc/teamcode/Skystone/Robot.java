package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.CurvePoint;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.lineCircleIntersection;

public class Robot {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    // Intake Motors
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    // Outtake Motors
    public DcMotor outtakeSpool;
    public DcMotor outtakeArm;

    // Outtake Arm Positions
    final int OUTTAKE_ARM_EXTENDED = -10;
    final int OUTTAKE_ARM_RESET = 0;

    // Outtake Servo
    public Servo clawServo;

    // Outtake Servo Positions
    final float CLAW_SERVO_CLAMPED = 1;
    final float CLAW_SERVO_RELEASED = -1;

    double i = 1;

    //robots position
    public Point robotPos = new Point();
    public double anglePos;

    //imu
    public BNO055IMU imu;
    public Orientation angles;
    public Position position;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    //dimensions
    public double wheelRadius = 2;
    public final double wheelCircumference = 4 * Math.PI;
    public final double encoderPerRevolution = 806.4;
    public final double l = 7;
    public final double w = 6.5;

    //PID (concept only)
    public double xMovement;
    public double yMovement;
    public double turnMovement;
    public double decelerationScaleFactor;

    public double pathDistance;

    public double distanceToEnd;

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

//        // Map intake motors
//        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
//        intakeRight = hardwareMap.dcMotor.get("intakeRight");
//
//        // Set direction of intake motors
//        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
//        intakeRight.setDirection(DcMotor.Direction.REVERSE);
//
//        // Map outtake motors
//        outtakeSpool = hardwareMap.dcMotor.get("outtakeSpool");
//        outtakeArm = hardwareMap.dcMotor.get("outtakeArm");
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

    //normal use method default 2 second kill time
    public void finalTurn(double targetHeading) {
        finalTurn(targetHeading, 2000);
    }

    public void finalTurn(double targetHeading, long timeInMilli) {
        targetHeading = Range.clip(targetHeading, -179, 179);
        long startTime = SystemClock.elapsedRealtime();
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        position = imu.getPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = startHeading - targetHeading;
        maxAngle = Math.abs(maxAngle);
        int sign = 0;
        if (targetHeading > startHeading) {
            sign = 1;
        } else {
            sign = -1;
        }
        if (maxAngle == 0) {
            return;
        }
        while (linearOpMode.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
            double scaleFactor = currentDeltatAngle / maxAngle;
            double absolutePower = 1 - scaleFactor;
            if (absolutePower < 0.1) {
                brakeRobot();
                return;
            }
            double power = 0.5 * absolutePower * sign;
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
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void allWheelDrive(double fLpower, double fRpower, double bLpower, double bRpower) {
        fLeft.setPower(fLpower);
        fRight.setPower(fRpower);
        bLeft.setPower(bLpower);
        bRight.setPower(bRpower);
    }

    public void intake (double intakeLeftPower, double intakeRightPower) {
        intakeLeft.setPower(intakeLeftPower);
        intakeRight.setPower(intakeRightPower);
    }

    public void outtake (char outtakeButton, double outtakeSpoolPower) {
        if (outtakeButton == 'a') {
            // Clamp and Extend
            clawServo.setPosition(CLAW_SERVO_CLAMPED);
            outtakeArm.setTargetPosition(OUTTAKE_ARM_EXTENDED);
        } else if (outtakeButton == 'b') {
            // Deposit and Reset
            clawServo.setPosition(CLAW_SERVO_RELEASED);
            outtakeArm.setTargetPosition(OUTTAKE_ARM_RESET);
        } else if (outtakeButton == 'x') {
            // Clamp
            clawServo.setPosition(CLAW_SERVO_CLAMPED);
        } else if (outtakeButton == 'y') {
            // Extend
            outtakeArm.setTargetPosition(OUTTAKE_ARM_EXTENDED);
        } else {
            telemetry.addData("status","there is a bug in robot.outtake() and it's ethan's fault");
            telemetry.update();
        }

        if (outtakeArm.getCurrentPosition() != outtakeArm.getTargetPosition()) {
            outtakeArm.setPower(1);
        } else {
            outtakeArm.setPower(0);
        }

        outtakeSpool.setPower(outtakeSpoolPower);
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

    //odometryUsingCircles
    //    public void odometryUsingCircles() {
//        double fLeftNEW = fLeft.getCurrentPosition();
//        double fRightNEW = fRight.getCurrentPosition();
//        double bLeftNEW = bLeft.getCurrentPosition();
//        double bRightNEW = bRight.getCurrentPosition();
//        double r;
//
//        double deltaLeft = wheelCircumference * (fLeftNEW-fLeftOLD)/encoderPerRevolution;
//        double deltaRight = wheelCircumference * (fRightNEW-fRightOLD)/encoderPerRevolution;
//
//        if (deltaRight == deltaLeft){
//            yDeltaRobot = deltaRight;
//        } else {
//            r = l * (deltaRight / (deltaLeft-deltaRight) + 1/2);
//            angleDeltaRobot = (deltaLeft-deltaRight)/14 * 0.51428571428;
//            xDeltaRobot = r * (1 - Math.cos(angleDeltaRobot));
//            yDeltaRobot = r * Math.sin(angleDeltaRobot);
//        }
//
//        //converting to global frame
//        xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
//        yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);
//        angleGlobal  = (wheelCircumference * (fLeftNEW)/encoderPerRevolution - wheelCircumference * (fRightNEW)/encoderPerRevolution) / 14 * 0.51428571428;
//
//        fLeftOLD = fLeftNEW;
//        fRightOLD = fRightNEW;
//        bLeftOLD = bLeftNEW;
//        bRightOLD = bRightNEW;
//    }

    public boolean followCurve(Vector<CurvePoint> allPoints, double followAngle, double speed) {
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
        if ((distanceToEnd < 1)) {
            return false;
        }
        if (distanceToEnd < 5) {
            i += 0.1;
        }
        i = Range.clip(i, 1, 2);
        applyMove(speed);
        return true;
    }

    public void moveFollowCurve(Vector<CurvePoint> points, double speed) {
        pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (linearOpMode.opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, Math.toRadians(0), speed)) {
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

    public CurvePoint getFollowPointPath(Vector<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
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

        double xStart = robotPos.x;
        double yStart = robotPos.y;
        double distanceTotal = Math.hypot(x - xStart, y - yStart);

        double distanceToTarget = Math.hypot(x - robotPos.x, y - robotPos.y);
        double absoluteAngleToTarget = Math.atan2(y - robotPos.y, x - robotPos.x);

        double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToPoint + optimalAngle;

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

//        decelerationScaleFactor = Range.clip(distanceToTarget / distanceTotal, -1, 1);

        xMovement = xPower * moveSpeed;
        yMovement = yPower * moveSpeed;
        turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
    }

    public void applyMove(double scaler) {
        double fLeftPower = (yMovement * 1.414 + turnMovement + xMovement);
        double fRightPower = (-yMovement * 1.414 - turnMovement + xMovement);
        double bLeftPower = (-yMovement * 1.414 + turnMovement + xMovement);
        double bRightPower = (yMovement * 1.414 - turnMovement + xMovement);

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

        scaler *= (distanceToEnd / pathDistance);
        scaler = Range.clip(scaler, 0.43, Integer.MAX_VALUE);

        fLeftPower *= scaler;
        fRightPower *= scaler;
        bLeftPower *= scaler;
        bRightPower *= scaler;

        double scaleDownAmount = 1.0;
        if (maxPower > 1.0) {
            scaleDownAmount = 1.0 / maxPower;
        }

        fLeftPower *= scaleDownAmount;
        fRightPower *= scaleDownAmount;
        bLeftPower *= scaleDownAmount;
        bRightPower *= scaleDownAmount;

        fLeftPower *= scaler;
        fRightPower *= scaler;
        bLeftPower *= scaler;
        bRightPower *= scaler;

        fLeft.setPower(fLeftPower);
        fRight.setPower(fRightPower);
        bLeft.setPower(bLeftPower);
        bRight.setPower(bRightPower);
    }

    public void moveToPoint(double x, double y, double moveSpeed, double turnSpeed, double optimalAngle) {
        double xStart = robotPos.x;
        double yStart = robotPos.y;
        double distanceTotal = Math.hypot(x - xStart, y - yStart);
        while (linearOpMode.opModeIsActive()) {
            double xPos = robotPos.x;
            double yPos = robotPos.y;
            double anglePos = this.anglePos;
            double distanceToTarget = Math.hypot(x - xPos, y - yPos);
            double absoluteAngleToTarget = Math.atan2(y - yPos, x - xPos);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
            double relativeTurnAngle = relativeAngleToPoint + optimalAngle;
            double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
            double xMovement = xPower * moveSpeed;
            double yMovement = yPower * moveSpeed;
            double turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double fLeftPower = (yMovement + turnMovement + xMovement * 1.414);
            double fRightPower = (-yMovement - turnMovement + xMovement * 1.414);
            double bLeftPower = (-yMovement + turnMovement + xMovement * 1.414);
            double bRightPower = (yMovement - turnMovement + xMovement * 1.414);
            telemetry.addLine("XPOS: " + xPos);
            telemetry.addLine("YPOS: " + yPos);
            telemetry.addLine("ANGPOS: " + Math.toDegrees(anglePos));
            telemetry.update();
            //op scaling
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
            fLeftPower *= Range.clip(distanceToTarget / distanceTotal, -1, 1);
            fRightPower *= Range.clip(distanceToTarget / distanceTotal, -1, 1);
            bLeftPower *= Range.clip(distanceToTarget / distanceTotal, -1, 1);
            bRightPower *= Range.clip(distanceToTarget / distanceTotal, -1, 1);
            double scaleDownAmount = 1.0;
            if (maxPower > 1.0) {
                scaleDownAmount = 1.0 / maxPower;
            }
            fLeftPower *= scaleDownAmount;
            fRightPower *= scaleDownAmount;
            bLeftPower *= scaleDownAmount;
            bRightPower *= scaleDownAmount;
            if (fLeftPower < 0.1 && fRightPower < 0.1 && bLeftPower < 0.1 && bRightPower < 0.1) {
                brakeRobot();
                return;
            }
            if (distanceToTarget < 0.75) {
                brakeRobot();
                return;
            }
            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);
        }
    }

//    public Point detectSkystone() {
//        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//        final boolean PHONE_IS_PORTRAIT = false;
//        final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";
//        final float mmPerInch = 25.4f;
//
//        final float stoneZ = 2.00f * mmPerInch;
//        Point point;
//                OpenGLMatrix lastLocation = null;
//                VuforiaLocalizer vuforia = null;
//                boolean targetVisible = false;
//                float phoneXRotate = 0;
//                float phoneYRotate = 0;
//                float phoneZRotate = 0;
//
//                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//                parameters.vuforiaLicenseKey = VUFORIA_KEY;
//                parameters.cameraDirection = CAMERA_CHOICE;
//                vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//                VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
//                VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//                stoneTarget.setName("Stone Target");
//                List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//                allTrackables.addAll(targetsSkyStone);
//                stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//                final float CAMERA_FORWARD_DISPLACEMENT = 9f * mmPerInch;
//                final float CAMERA_VERTICAL_DISPLACEMENT = 9f * mmPerInch;
//                final float CAMERA_LEFT_DISPLACEMENT = -4;
//
//                OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//                ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//                targetsSkyStone.activate();
//
//                while(true) {
//
//                    // check all the trackable targets to see which one (if any) is visible.
//                    targetVisible = false;
//                    for (VuforiaTrackable trackable : allTrackables) {
//                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                            telemetry.addData("Visible Target", trackable.getName());
//                            targetVisible = true;
//
//                            // getUpdatedRobotLocation() will return null if no new information is available since
//                            // the last time that call was made, or if the trackable is not currently visible.
//                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                            if (robotLocationTransform != null) {
//                                lastLocation = robotLocationTransform;
//                            }
//                            break;
//                        }
//                    }
//                    if (targetVisible) {
//                        // express position (translation) of robot in inches.
//                        VectorF translation = lastLocation.getTranslation();
//                        return new Point(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
//
//                        // express the rotation of the robot in degrees.
//                    } else {
//                        telemetry.addData("Visible Target", "none");
//                    }
//                    telemetry.update();
//                }
//    }
}