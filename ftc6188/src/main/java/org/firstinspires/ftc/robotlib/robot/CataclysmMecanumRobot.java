package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.armsystem.FieldGoalArmSystem;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.motor.EncoderMotor;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;
import org.firstinspires.ftc.robotlib.servo.LinkedStateServo;
import org.firstinspires.ftc.robotlib.servo.StateServo;
import org.firstinspires.ftc.robotlib.sound.BasicSound;
import org.firstinspires.ftc.robotlib.state.ServoState;

public class CataclysmMecanumRobot
{
    // Drive motors
    private EncoderMotor driveFrontLeft;
    private EncoderMotor driveFrontRight;
    private EncoderMotor driveRearLeft;
    private EncoderMotor driveRearRight;
    private EncoderMotor[] driveMotorList;

    // Drive constants
    private static final double WHEEL_RADIUS_IN = 2;
    private static final double MOTOR_TO_WHEEL_RATIO = 2;

    // Arm motors
    public EncoderMotor armIntakeLeft;
    public EncoderMotor armIntakeRight;

    // Servo motors
    public StateServo servoDeliveryLeft;
    public StateServo servoDeliveryRight;
    public StateServo servoBlockGrabber;

    // Telemetry reference
    private final Telemetry telemetry;

    // Movement systems (drivetrain/arm system)
    public MecanumDrivetrain drivetrain;

    public CataclysmMecanumRobot(HardwareMap hwMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        // Drive motors init
        driveFrontLeft = new EncoderMotor(hwMap.get(DcMotor.class, "driveFrontLeft"));
        driveFrontRight = new EncoderMotor(hwMap.get(DcMotor.class, "driveFrontRight"));
        driveRearRight = new EncoderMotor(hwMap.get(DcMotor.class, "driveRearRight"));
        driveRearLeft = new EncoderMotor(hwMap.get(DcMotor.class, "driveRearLeft"));
        driveMotorList = new EncoderMotor[] {driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        // Since all drive motors are considered the same a loop is used
        for (DcMotor motor : driveMotorList)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Arm motors init
        armIntakeLeft = new EncoderMotor(hwMap.get(DcMotor.class, "intakeLeft"));
        armIntakeRight = new EncoderMotor(hwMap.get(DcMotor.class, "intakeRight"));

        armIntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armIntakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armIntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos init
        servoDeliveryLeft = new StateServo(hwMap.get(Servo.class, "deliveryLeft"), 0, 0, 1);
        servoDeliveryRight = new StateServo(hwMap.get(Servo.class, "deliveryRight"), 1, 1, 0);
        servoBlockGrabber = new StateServo(hwMap.get(Servo.class, "blockGrabber"), 1, 1, 0);

        servoDeliveryLeft.setDirection(Servo.Direction.FORWARD);
        servoDeliveryRight.setDirection(Servo.Direction.FORWARD);
        servoBlockGrabber.setDirection(Servo.Direction.FORWARD);

        servoDeliveryLeft.setPosition(ServoState.STOWED);
        servoDeliveryRight.setPosition(ServoState.STOWED);
        servoBlockGrabber.setPosition(ServoState.STOWED);

        // Systems init
        drivetrain = new MecanumDrivetrain(driveMotorList, WHEEL_RADIUS_IN, MOTOR_TO_WHEEL_RATIO);
    }

    public void informationTelemetry()
    {
        telemetry.addData("> Target Positions", "-----");
        telemetry.addData("WheelTarget FL", drivetrain.wheelTargetPositions[0]);
        telemetry.addData("WheelTarget FR", drivetrain.wheelTargetPositions[1]);
        telemetry.addData("WheelTarget RL", drivetrain.wheelTargetPositions[2]);
        telemetry.addData("WheelTarget RR", drivetrain.wheelTargetPositions[3]);
        telemetry.addData("Distance Target", drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", drivetrain.motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", drivetrain.motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", drivetrain.motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", drivetrain.motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos Percent", drivetrain.getCurrentPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition() * drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Motor Revs Per Sec", "-----");
        telemetry.addData("Speed FL", drivetrain.motorList[0].getRevolutionsPerSecond());
        telemetry.addData("Speed FR", drivetrain.motorList[1].getRevolutionsPerSecond());
        telemetry.addData("Speed RL", drivetrain.motorList[2].getRevolutionsPerSecond());
        telemetry.addData("Speed RR", drivetrain.motorList[3].getRevolutionsPerSecond());

        telemetry.addData("> Is Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isBusy());

        telemetry.addData("> Is Encoder Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isEncoderBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isEncoderBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isEncoderBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isEncoderBusy());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());
        telemetry.addData("Is Pos", drivetrain.isPositioning());

        telemetry.addData("Arm RPM", "-----");
        telemetry.addData("Left", armIntakeLeft.getRevolutionsPerSecond());
        telemetry.addData("Right", armIntakeRight.getRevolutionsPerSecond());

        telemetry.update();
    }
}
