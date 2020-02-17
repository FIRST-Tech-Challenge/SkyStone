package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {

    //Drive Motors
    public static DcMotor rightFront, leftFront, leftBack, rightBack; //Drive Motor Objects
    //IMU
    public static BNO055IMU imu;
    //Intake Motors
    public static DcMotor intakeLeft, intakeRight;

    public static void hardwareMap(HardwareMap hardwareMap) {

        rightFront = hardwareMap.dcMotor.get("driveFrontRight");
        leftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        leftBack = hardwareMap.dcMotor.get("driveBackLeft");
        rightBack = hardwareMap.dcMotor.get("driveBackRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }

    public static void testHardware(){
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setPower(0.3);
    }
}
