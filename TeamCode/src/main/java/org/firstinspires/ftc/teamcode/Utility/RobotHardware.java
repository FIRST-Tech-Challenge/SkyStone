package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {

    private HardwareMap hardwareMap;
    //Drive Motors
    public DcMotor rightFront, leftFront, leftBack, rightBack; //Drive Motor Objects
    //IMU
    public BNO055IMU imu;
    //Intake Motors
    public DcMotor intakeLeft, intakeRight;

    public RobotHardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

    }

    public void hardwareMap() {

        rightFront = hardwareMap.dcMotor.get("driveFrontRight");
        leftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        leftBack = hardwareMap.dcMotor.get("driveBackLeft");
        rightBack = hardwareMap.dcMotor.get("driveBackRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }
}
