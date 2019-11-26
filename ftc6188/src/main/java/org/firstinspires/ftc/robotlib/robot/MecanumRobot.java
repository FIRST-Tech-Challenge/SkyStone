package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.servo.LinkedServo;

public class MecanumRobot
{
    // Drive motors
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;
    private DcMotor[] motorList;

    // Platform servos
    private Servo servoClawLeft;
    private Servo servoClawRight;

    // Drive constants
    private static final double wheelRadius = 2; //inches
    private static final double wheelToMotorRatio = 2.0/1.0;

    // Temporary telemetry reference will likely be removed later
    Telemetry telemetry;

    // This robot operates on a mecanum drivetrain
    public MecanumDrivetrain drivetrain;

    // The various LinkedServo or LimitedMotors used in place of regular items
    public LinkedServo platformServos;

    // This general constructor style should be followed with all Robots
    public MecanumRobot(HardwareMap hwMap, final Telemetry telemetry, boolean teleOpMode)
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

        // Construct the drive and arm systems
        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};
        drivetrain = new MecanumDrivetrain(motorList, teleOpMode, wheelRadius, wheelToMotorRatio);
        platformServos = new LinkedServo(servoClawLeft, servoClawRight);

        // Telemetry Initialization
        telemetry.addAction(new Runnable()
        {
            @Override
            public void run()
            {

            }
        });
        // front left
        telemetry.addLine()
                .addData("driveFrontLeft: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("");
                    }
                })
                .addData("IsBusy: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontLeft.isBusy());
                    }
                })
                .addData("Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontLeft.getCurrentPosition());
                    }
                })
                .addData("Target Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontLeft.getTargetPosition());
                    }
                })
                .addData("Power: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontLeft.getPower());
                    }
                });
        // front right
        telemetry.addLine()
                .addData("driveFrontRight: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("");
                    }
                })
                .addData("IsBusy: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontRight.isBusy());
                    }
                })
                .addData("Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontRight.getCurrentPosition());
                    }
                })
                .addData("Target Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontRight.getTargetPosition());
                    }
                })
                .addData("Power: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveFrontRight.getPower());
                    }
                });
        // rear left
        telemetry.addLine()
                .addData("driveRearLeft: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("");
                    }
                })
                .addData("IsBusy: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearLeft.isBusy());
                    }
                })
                .addData("Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearLeft.getCurrentPosition());
                    }
                })
                .addData("Target Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearLeft.getTargetPosition());
                    }
                })
                .addData("Power: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearLeft.getPower());
                    }
                });
        // rear right
        telemetry.addLine()
                .addData("driveRearRight: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("");
                    }
                })
                .addData("IsBusy: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearRight.isBusy());
                    }
                })
                .addData("Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearRight.getCurrentPosition());
                    }
                })
                .addData("Target Position: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearRight.getTargetPosition());
                    }
                })
                .addData("Power: ", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return ("" + driveRearRight.getPower());
                    }
                });
    }

    // Just a central place to store all the telemetry from the robot, will likely me removed later
    public void informationUpdate()
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

        telemetry.addData("> Is Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isBusy());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());
        telemetry.addData("Is Pos", drivetrain.isPositioning());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Servo Pos", "One: " + platformServos.getServoOne().getPosition() + " Two: " + platformServos.getServoTwo().getPosition());
        telemetry.addData("Servo Pos2", "One: " + servoClawLeft.getPosition() + " Two: " + servoClawRight.getPosition());
        telemetry.addData("Linked Pos", platformServos.getPosition());
    }
}
