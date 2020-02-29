package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //Drive Motors
    public DcMotor rightFront, leftFront, leftBack, rightBack; //Drive Motor Objects
    //IMU
    public BNO055IMU imu;
    //Intake Motors
    public DcMotor intakeLeft, intakeRight;
    //Foundation Clamps
    public Servo foundationClampLeft, foundationClampRight;
    //Auto Block Arms
    public Servo autoFlipperLeft, autoFlipperRight, autoGrabberLeft, autoGrabberRight;
    //Lift Motors
    public DcMotor liftRight, liftLeft;
    //Block Clamp
    public Servo blockGrabberFront, blockGrabberBack;
    //Outtake Flipper
    public Servo flipperServoLeft, flipperServoRight;

    public void hardwareMap(HardwareMap hardwareMap) {

        //Drive-train
        rightFront = hardwareMap.dcMotor.get("driveFrontRight");
        leftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        leftBack = hardwareMap.dcMotor.get("driveBackLeft");
        rightBack = hardwareMap.dcMotor.get("driveBackRight");
        //Intake
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        //IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(Params);
        //Auto arms
        autoFlipperLeft = hardwareMap.servo.get("autoFlipperLeft");
        autoFlipperRight = hardwareMap.servo.get("autoFlipperRight");
        autoGrabberLeft = hardwareMap.servo.get("autoGrabberLeft");
        autoGrabberRight = hardwareMap.servo.get("autoGrabberRight");
        //Foundation Clamp
        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");
        //Block Clamp
        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");
        //Outtake Flipper
        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");
        //Extrusion Motors
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

    }

    public void testHardware(){
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setPower(0.3);
    }
}
