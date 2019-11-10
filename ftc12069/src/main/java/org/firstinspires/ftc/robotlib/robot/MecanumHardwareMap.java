package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.servo.ServoManager;

public class MecanumHardwareMap
{
    public HardwareMap internalHardwareMap;

    // Motors in Mecanum robot
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    // Servos in Mecanum Robot
    private Servo servoClaw;

    public MecanumDrivetrain drivetrain;
    public ServoManager servoManager;

    public final double wheelRadius = 4; //inches
    private static final double wheelToMotorRatio = 1.0/1.0;

    public DcMotor[] motorList;

    public final double motorTicksPerInch;

    /**
     * Creates a mecanum hardware map from the FTC given hardware map
     * @param hwMap FTC hardware map
     */
    public MecanumHardwareMap(HardwareMap hwMap)
    {
        this.internalHardwareMap = hwMap;

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

        driveFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        servoClaw = hwMap.get(Servo.class, "servoClaw");
        servoClaw.setDirection(Servo.Direction.FORWARD);

        drivetrain = new MecanumDrivetrain(motorList);
        servoManager = new ServoManager(new Servo[]{servoClaw});
        motorTicksPerInch = drivetrain.getTicksPerInch(wheelRadius, wheelToMotorRatio);
    }
}
