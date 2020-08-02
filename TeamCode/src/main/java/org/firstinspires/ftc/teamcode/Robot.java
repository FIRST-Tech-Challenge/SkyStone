package org.firstinspires.ftc.teamcode;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

public class Robot {
    public OdometryGlobalCoordinatePosition pt;
    public DcMotor topLeft;
    public DcMotor topRight;
    public DcMotor botLeft;
    public DcMotor botRight;
    public DcMotor intakeR;
    public DcMotor intakeL;

    private DcMotor winchL;
    private DcMotor winchR;
    private Servo pullL;
    private Servo pullR;

    private Servo clawF;
    private Servo clawB;

    private Servo armL;
    private Servo armR;

    private Servo intakeServoL;
    private Servo intakeServoR;

    private DistanceSensor blockDistance;

    BNO055IMU imu;
    Orientation orientation = new Orientation();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation, constant;
    PIDController pidRotate, pidDrive, pidStrafe, pidCurve, pidCorrection;
    DcMotor.RunMode newRun;
    HardwareMap map;
    Rectangle frame;
    private boolean auto;
    public Robot(DcMotor.RunMode runMode, HardwareMap imported, double x, double y, double robotLength, double robotWidth) {
        auto = true;
        map = imported;
        newRun = runMode;
        topLeft = map.dcMotor.get("topLeft");
        topRight = map.dcMotor.get("topRight");
        botLeft = map.dcMotor.get("botLeft");
        botRight = map.dcMotor.get("botRight");
        winchL = map.dcMotor.get("winchL");
        winchR = map.dcMotor.get("winchR");

        intakeL = map.dcMotor.get("intakeL");
        intakeR = map.dcMotor.get("intakeR");

        pullL = map.servo.get("pullL");
        pullR = map.servo.get("pullR");

        armL = map.servo.get("armL");
        armR = map.servo.get("armR");

        clawF = map.servo.get("clawF");
        clawB = map.servo.get("clawB");


        intakeServoL = map.servo.get("intakeServoL");
        intakeServoR = map.servo.get("intakeServoR");

        blockDistance = map.get(DistanceSensor.class, "distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //set reading to degrees

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);//axes order based on revHub orientation
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        pidRotate = new PIDController(.013, 0, 0.00007);
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls samhow sensitive the correction is.
        pidDrive = new PIDController(0.009, 0.00006, 0.00008);

        pidCurve = new PIDController(0.5, 0, 0.0007);
        //pidStrafe = new PIDController(0, 0, 0);
        pidStrafe = new PIDController(0.008, 0.0004, 0);
        pidCorrection = new PIDController(0,0,0);

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            idle();
            sleep(50);
        }
        constant = getHeading();//constant is meant to compensate for any difference marked at the beginning. We should be starting at 0 manually
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);//set to imported runMode
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);

        winchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);//in reverse because of motor direction
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bot = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 560, 760, x, y);
        pt  = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 307.699557, 760, x, y);
        frame = new Rectangle(robotLength, robotWidth);
    }
    public void init(){
       Thread newThread = new Thread(pt);
       newThread.start();
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void resetAngle()//for PID
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle()//for PID
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getHeading() {//for overall
//        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double val = Double.parseDouble(formatAngle(orientation.angleUnit, orientation.firstAngle)) - constant;
//        if (val > 180) {
//            val -= 360;
//        } else if (val < -180) {
//            val += 360;
//        }
//        return val;
        return pt.returnOrientation();
    }
    public void resetHeading(){
        constant = getHeading();
    }
    public final void idle() {
        Thread.yield();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public void initialDown() {
        armL.setPosition(.3);
        armR.setPosition(.7);
        intakeServoL.setPosition(0.1);
        intakeServoR.setPosition(.85);
    }

    public void pullDown() {
        pullL.setPosition(.46);
        pullR.setPosition(.49);
    }

    public void pullUp() {
        pullL.setPosition(.9);
        pullR.setPosition(0.05);
    }

    public void pullMid() {
        pullL.setPosition(.6);
        pullR.setPosition(.2);
    }

    public void intake() {
        intakeL.setPower(1);
        intakeR.setPower(-1);

    }

    public void intakeOff() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
}