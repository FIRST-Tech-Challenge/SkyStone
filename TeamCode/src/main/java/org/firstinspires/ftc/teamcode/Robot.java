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



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //set reading to degrees
        pidRotate = new PIDController(.013, 0, 0.00007);
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls samhow sensitive the correction is.
        pidDrive = new PIDController(0.009, 0.00006, 0.00008);

        pidCurve = new PIDController(0.5, 0, 0.0007);
        //pidStrafe = new PIDController(0, 0, 0);
        pidStrafe = new PIDController(0.008, 0.0004, 0);
        pidCorrection = new PIDController(0,0,0);

        // make sure the imu gyro is calibrated before continuing.
        constant = getHeading();//constant is meant to compensate for any difference marked at the beginning. We should be starting at 0 manually
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);//set to imported runMode
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);


        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);//in reverse because of motor direction
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
        return pt.returnOrientation() - constant;
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
}