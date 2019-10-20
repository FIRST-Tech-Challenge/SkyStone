package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;

public class MecanumRobot
{
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    public DcMotor armParallelLift;

    public Servo servoBuildClawLeft;
    public Servo servoBuildClawRight;

    public MecanumDrivetrain drivetrain;
    public DcMotor[] motorList;

    public final double wheelRadius = 2; //inches
    public final double gearRatio = (1.0/2.0);

    public MecanumRobot(HardwareMap hwMap)
    {
        driveFrontLeft = hwMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hwMap.get(DcMotor.class, "driveFrontRight");
        driveRearRight = hwMap.get(DcMotor.class, "driveRearRight");
        driveRearLeft = hwMap.get(DcMotor.class, "driveRearLeft");

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        /**
        armParallelLift = hwMap.get(DcMotor.class, "armParallelLift");
        armParallelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armParallelLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armParallelLift.setDirection(DcMotorSimple.Direction.FORWARD);
         **/

        servoBuildClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoBuildClawRight = hwMap.get(Servo.class, "servoClawRight");

        servoBuildClawLeft.setDirection(Servo.Direction.FORWARD);
        servoBuildClawRight.setDirection(Servo.Direction.FORWARD);

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};
        drivetrain = new MecanumDrivetrain(motorList);
    }
}
