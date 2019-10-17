package org.firstinspires.ftc.robotlib.hardwaremap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;

public class MecanumHardwareMap extends HardwareMapTemplate
{
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    public DcMotor armParallelLift;

    public Servo servoBuildClawLeft;
    public Servo servoBuildClawRight;

    public final double wheelRadius = 4; //inches

    public MecanumHardwareMap(HardwareMap hwMap)
    {
        driveFrontLeft = hwMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hwMap.get(DcMotor.class, "driveFrontRight");
        driveRearRight = hwMap.get(DcMotor.class, "driveRearRight");
        driveRearLeft = hwMap.get(DcMotor.class, "driveRearLeft");

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
        armParallelLift = hwMap.get(DcMotor.class, "armParallelLift");
        armParallelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armParallelLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armParallelLift.setDirection(DcMotorSimple.Direction.FORWARD);
         **/

        /**
        servoBuildClawLeft = hwMap.get(Servo.class, "servoBuildClawLeft");
        servoBuildClawRight = hwMap.get(Servo.class, "servoBuildClawRight");

        servoBuildClawLeft.setDirection(Servo.Direction.FORWARD);
        servoBuildClawRight.setDirection(Servo.Direction.FORWARD);
         **/

        drivetrain = new MecanumDrivetrain(motorList);
    }
}
