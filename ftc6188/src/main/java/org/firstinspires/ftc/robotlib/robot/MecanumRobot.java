package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.servo.LinkedServo;

public class MecanumRobot
{
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    private Servo servoBuildClawLeft;
    private Servo servoBuildClawRight;

    public MecanumDrivetrain drivetrain;
    public LinkedServo platformServos;

    public DcMotor[] motorList;

    public final double wheelRadius = 2; //inches
    public final double motorToWheelRatio = (1.0/2.0);
    public double motorTicksPerIn = (1.0/((wheelRadius*2*Math.PI) * motorToWheelRatio));

    public MecanumRobot(HardwareMap hwMap, boolean isAutoMode)
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

        servoBuildClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoBuildClawRight = hwMap.get(Servo.class, "servoClawRight");

        servoBuildClawLeft.setDirection(Servo.Direction.FORWARD);
        servoBuildClawRight.setDirection(Servo.Direction.REVERSE);

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};
        drivetrain = new MecanumDrivetrain(motorList, isAutoMode);
        platformServos = new LinkedServo(servoBuildClawLeft, servoBuildClawRight);

        motorTicksPerIn *= drivetrain.motorList[0].getMotorType().getTicksPerRev();
    }
}
