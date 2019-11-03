package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.servo.LinkedServo;

public class MecanumRobot
{
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;
    DcMotor[] motorList;

    private Servo servoClawLeft;
    private Servo servoClawRight;

    private static final double wheelRadius = 2; //inches
    private static final double wheelToMotorRatio = 2.0/1.0;

    private Telemetry telemetry;

    public MecanumDrivetrain drivetrain;
    public LinkedServo platformServos;

    public final double motorTicksPerIN;

    public MecanumRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
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

        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");

        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.REVERSE);

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        drivetrain = new MecanumDrivetrain(motorList, teleOpMode);
        platformServos = new LinkedServo(servoClawLeft, servoClawRight);

        motorTicksPerIN = drivetrain.getTicksPerIn(wheelRadius, wheelToMotorRatio);
    }

    public void robotMove(double course, double velocity, double rotation, double distance)
    {
        drivetrain.setCourse(course * Math.PI/180); //converts a degree input into radians
        drivetrain.setVelocity(velocity * 0.25); //quarters the velocity since a high velocity causes massive drift following a move command
        drivetrain.setRotation(rotation);
        drivetrain.setTargetPosition(distance * motorTicksPerIN); // adjust a distance in inches to the appropriate amount of motor ticks
        informationUpdate();
    }

    public void informationUpdate()
    {
        telemetry.addData("Ticks Per IN", motorTicksPerIN);
        telemetry.addData("WheelTarget FL", drivetrain.motorList[0].getTargetPosition());
        telemetry.addData("WheelTarget FR", drivetrain.motorList[1].getTargetPosition());
        telemetry.addData("WheelTarget RL", drivetrain.motorList[2].getTargetPosition());
        telemetry.addData("WheelTarget RR", drivetrain.motorList[3].getTargetPosition());
        telemetry.addData("Course", drivetrain.getCourse());
        telemetry.addData("Velocity", drivetrain.getVelocity());
        telemetry.addData("Rotation", drivetrain.getRotation());
        telemetry.addData("Distance", drivetrain.getTargetPosition());
        telemetry.addData("Servo Pos", platformServos.getActual());
        telemetry.addData("Linked Pos", platformServos.getPosition());
        telemetry.update();
    }
}
