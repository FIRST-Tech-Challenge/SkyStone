package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;

public class EncoderTestPlatformRobot
{
    public DcMotor driveFrontLeft;
    public DcMotor driveFrontRight;
    public DcMotor driveRearLeft;
    public DcMotor driveRearRight;
    private DcMotor[] driveMotorList;

    public final int REVOLUTION;

    public MecanumDrivetrain drivetrain;

    public EncoderTestPlatformRobot(HardwareMap hwMap)
    {
        // Drive motors init
        driveFrontLeft = hwMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hwMap.get(DcMotor.class, "driveFrontRight");
        driveRearRight = hwMap.get(DcMotor.class, "driveRearRight");
        driveRearLeft = hwMap.get(DcMotor.class, "driveRearLeft");
        REVOLUTION = (int)driveFrontLeft.getMotorType().getTicksPerRev();
        driveMotorList = new DcMotor[] {driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        for (DcMotor motor : driveMotorList)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveRearRight.setDirection(DcMotor.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotor.Direction.FORWARD);

        drivetrain = new MecanumDrivetrain(driveMotorList);
    }

    public void informationTelemetry (Telemetry telemetry)
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
        telemetry.addData("Current Pos Percent", drivetrain.getCurrentPosition()/drivetrain.getTargetPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Is Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isBusy());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180.0);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());
        telemetry.addData("Is Pos", drivetrain.isPositioning());

        telemetry.update();
    }

    public void informationTelemetry(Telemetry telemetry, String programStatus)
    {
        telemetry.addData("> Program Status", programStatus);
        informationTelemetry(telemetry);
    }
}
