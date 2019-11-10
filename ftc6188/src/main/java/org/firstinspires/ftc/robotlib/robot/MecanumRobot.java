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

    static final double wheelRadius = 2; //inches
    static final double wheelToMotorRatio = 2.0/1.0;

    private Telemetry telemetry;

    public MecanumDrivetrain drivetrain;
    public LinkedServo platformServos;

    public MecanumRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        this.telemetry = telemetry;

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

        drivetrain = new MecanumDrivetrain(new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight},
                teleOpMode, wheelRadius, wheelToMotorRatio);
        platformServos = new LinkedServo(servoClawLeft, servoClawRight);
    }

    public void autoPosition(double course, double velocity, double rotation, double distance) // distance in inches
    {
        drivetrain.setCourse(course * Math.PI/180); //converts a degree input into radians
        drivetrain.setAutoVelocity(velocity);
        drivetrain.setRotation(rotation);
        drivetrain.setTargetPosition(distance * drivetrain.getTicksPerIn()); // adjust a distance in inches to the appropriate amount of motor ticks

        while (drivetrain.isPositioning())
        {
            telemetry.addData("Auto Status", "In Pos Loop");
            drivetrain.updatePosition();
            informationUpdate();
        }

        telemetry.addData("Auto Status", "In Fin");
        drivetrain.finishPositioning();
    }

    public void informationUpdate()
    {
        telemetry.addData("> Target Positions", "-----");
        telemetry.addData("WheelTarget FL", drivetrain.motorList[0].getTargetPosition());
        telemetry.addData("WheelTarget FR", drivetrain.motorList[1].getTargetPosition());
        telemetry.addData("WheelTarget RL", drivetrain.motorList[2].getTargetPosition());
        telemetry.addData("WheelTarget RR", drivetrain.motorList[3].getTargetPosition());
        telemetry.addData("Distance Target", drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", drivetrain.motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", drivetrain.motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", drivetrain.motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", drivetrain.motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Movement Velocity", drivetrain.getAutoVelocity());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Servo Pos", platformServos.getActual());
        telemetry.addData("Linked Pos", platformServos.getPosition());
        telemetry.update();
    }
}
