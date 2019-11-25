package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.motor.FakeMotor;

public class FakeMecanumRobot
{
    // Motors and list
    private FakeMotor driveFrontLeft;
    private FakeMotor driveFrontRight;
    private FakeMotor driveRearRight;
    private FakeMotor driveRearLeft;
    private DcMotor[] motorList;
    private FakeMotor[] fakeMotorList;

    // Telemetry reference
    private Telemetry telemetry;

    // Drive systems
    public MecanumDrivetrain drivetrain;

    public FakeMecanumRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        // Init fake motors
        driveFrontLeft = new FakeMotor("driveFrontLeft");
        driveFrontRight = new FakeMotor("driveFrontRight");
        driveRearLeft = new FakeMotor("driveRearLeft");
        driveRearRight = new FakeMotor("driveRearRight");
        fakeMotorList = new FakeMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};
        motorList = fakeMotorList;

        drivetrain = new MecanumDrivetrain(motorList, teleOpMode);

        // Init telemetry
        telemetry.addLine()
                .addData("Front Left: ", new Func<String>()
                {
                    @Override public String value()
                    {
                        return "";
                    }
                })
                .addData("Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontLeft.getPower());
                    }
                })
                .addData("Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontLeft.getCurrentPosition());
                    }
                })
                .addData("Target Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontLeft.getTargetPosition());
                    }
                })
                .addData("Run Mode", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontLeft.getMode());
                    }
                });
        telemetry.addLine()
                .addData("Front Right: ", new Func<String>()
                {
                    @Override public String value()
                    {
                        return "";
                    }
                })
                .addData("Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontRight.getPower());
                    }
                })
                .addData("Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontRight.getCurrentPosition());
                    }
                })
                .addData("Target Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontRight.getTargetPosition());
                    }
                })
                .addData("Run Mode", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveFrontRight.getMode());
                    }
                });
        telemetry.addLine()
                .addData("Rear Left: ", new Func<String>()
                {
                    @Override public String value()
                    {
                        return "";
                    }
                })
                .addData("Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearLeft.getPower());
                    }
                })
                .addData("Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearLeft.getCurrentPosition());
                    }
                })
                .addData("Target Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearLeft.getTargetPosition());
                    }
                })
                .addData("Run Mode", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearLeft.getMode());
                    }
                });
        telemetry.addLine()
                .addData("Rear Right: ", new Func<String>()
                {
                    @Override public String value()
                    {
                        return "";
                    }
                })
                .addData("Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearRight.getPower());
                    }
                })
                .addData("Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearRight.getCurrentPosition());
                    }
                })
                .addData("Target Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearRight.getTargetPosition());
                    }
                })
                .addData("Run Mode", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + driveRearRight.getMode());
                    }
                });
    }
}
