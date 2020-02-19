package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //Drive Motors
    public static DcMotor rightFront, leftFront, leftBack, rightBack; //Drive Motor Objects
    //IMU
    public static BNO055IMU imu;
    //Intake Motors
    public static DcMotor intakeLeft, intakeRight;
    //Foundation Clamps
    public static Servo foundationClampLeft, foundationClampRight;
    //Auto Block Arms
    public static Servo autoFlipperLeft, autoFlipperRight, autoGrabberLeft, autoGrabberRight;

    public static void hardwareMap(HardwareMap hardwareMap) {

        //Drive-train
        rightFront = hardwareMap.dcMotor.get("driveFrontRight");
        leftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        leftBack = hardwareMap.dcMotor.get("driveBackLeft");
        rightBack = hardwareMap.dcMotor.get("driveBackRight");
        //Intake
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        //IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Auto arms
        autoFlipperLeft = hardwareMap.servo.get("autoFlipperLeft");
        autoFlipperRight = hardwareMap.servo.get("autoFlipperRight");
        autoGrabberLeft = hardwareMap.servo.get("autoGrabberLeft");
        autoGrabberRight = hardwareMap.servo.get("autoGrabberRight");
        //Foundation Clamp
        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");

    }

    public static void testHardware(){
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setPower(0.3);
    }
}
