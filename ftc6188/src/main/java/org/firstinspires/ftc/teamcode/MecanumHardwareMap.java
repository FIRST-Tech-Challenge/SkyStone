package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumHardwareMap
{
    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveRearRight;
    DcMotor driveRearLeft;

    public DcMotor[] motorList;
    public double[] wheelAngles;
    public double[] wheelTargetPositions;
    public DcMotor.RunMode[] runModes;

    private HardwareMap chwMap;

    MecanumHardwareMap(HardwareMap hwMap)
    {
        chwMap = hwMap;
    }

    public void init()
    {
        driveFrontLeft = chwMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = chwMap.get(DcMotor.class, "driveFrontRight");
        driveRearRight = chwMap.get(DcMotor.class, "driveRearRight");
        driveRearLeft = chwMap.get(DcMotor.class, "driveRearLeft");

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};
        wheelAngles = new double[]{-3*Math.PI/4, 3*Math.PI/4, -Math.PI/4, Math.PI/4};
        wheelTargetPositions = new double[4];
        runModes = new DcMotor.RunMode[4];

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
