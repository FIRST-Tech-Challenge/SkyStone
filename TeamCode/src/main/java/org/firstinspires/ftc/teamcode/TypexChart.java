package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TypexChart {
    public DcMotor TL, TR, BL, BR;
    //public BNO055IMU imu;
    public Servo hookLeft, hookRight;

    public DistanceSensor distanceSensor;

    HardwareMap chart;
    ElapsedTime runtime = new ElapsedTime();

    public TypexChart()
    {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        CONSTANTS constants = new CONSTANTS();
        chart = ahwMap;

        // Define and Initialize Motors
        TL = ahwMap.get(DcMotor.class, "TL");
        TR = ahwMap.get(DcMotor.class, "TR");
        BL = ahwMap.get(DcMotor.class, "BL");
        BR = ahwMap.get(DcMotor.class, "BR");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        //imu = ahwMap.get(BNO055IMU.class, "imu");

        hookLeft = ahwMap.get(Servo.class, "hookLeft");
        hookRight = ahwMap.get(Servo.class, "hookRight");

        // Set all motors to zero power
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        hookLeft.setPosition(constants.OPENPOSITION);
        hookRight.setPosition(constants.OPENPOSITION);

        TL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        distanceSensor = ahwMap.get(DistanceSensor.class, "dist");
    }
}
