package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.managers.MotorManager;
import org.firstinspires.ftc.robotlib.managers.ServoManager;

public class MecanumHardwareMap
{
    public HardwareMap internalHardwareMap;

    // Motors in Mecanum robot
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    // Intake
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Servos
    public Servo blockGrabber;
    private Servo deliveryLeft;
    private Servo deliveryRight;

    // Camera
    public WebcamName webcamName;

    // REV IMU
    public BNO055IMU imu;

    public MecanumDrivetrain drivetrain;

    // Managers (for treating a group of objects as one)
    public ServoManager deliveryServoManager;
    public MotorManager intakeMotorManager;

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

        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");

        blockGrabber = hwMap.get(Servo.class, "blockGrabber");
        deliveryLeft = hwMap.get(Servo.class, "deliveryLeft");
        deliveryRight = hwMap.get(Servo.class, "deliveryLeft");

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

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

        blockGrabber.setDirection(Servo.Direction.FORWARD);
        deliveryLeft.setDirection(Servo.Direction.FORWARD);
        deliveryRight.setDirection(Servo.Direction.FORWARD);

        webcamName = hwMap.get(WebcamName.class, "webcam");

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; If we find problems manual calibration will be required
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        drivetrain = new MecanumDrivetrain(motorList);
        intakeMotorManager = new MotorManager(new DcMotor[]{intakeLeft, intakeRight});
        deliveryServoManager = new ServoManager(new Servo[] {deliveryLeft, deliveryRight});

        motorTicksPerInch = drivetrain.getTicksPerInch(wheelRadius, wheelToMotorRatio);
    }
}
