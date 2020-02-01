package org.firstinspires.ftc.teamcode.All;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class HardwareMap {
    public DcMotorEx backLeft, backRight, frontLeft, frontRight, leftIntake, rightIntake, liftOne, liftTwo;
    public Servo clawServo1, clawServo2, foundationLock, transferLock, transferHorn,
            clawInit, innerTransfer, intakeInit, redAutoClawJoint1, redAutoClawJoint2, redAutoClawJoint3,
            parkingServo, liftOdometry;
    public DigitalChannel liftReset, intakeDetect, foundationDetectLeft, foundationDetectRight;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public static String TAG = "MainThread";

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {

        //region
        //------------------------===Drive Motors===------------------------
        backLeft = (DcMotorEx) hwMap.get(DcMotor.class, "backLeft");
        backRight = (DcMotorEx) hwMap.get(DcMotor.class, "backRight");
        frontLeft = (DcMotorEx) hwMap.get(DcMotor.class, "frontLeft");
        frontRight = (DcMotorEx) hwMap.get(DcMotor.class, "frontRight");
        //---------------------------------------------------------------------------
        //endregion

        //region
        //------------------------===Intake & Lift Motors===------------------------

        leftIntake = (DcMotorEx) hwMap.get(DcMotor.class, "leftIntake");
        rightIntake = (DcMotorEx) hwMap.get(DcMotor.class, "rightIntake");
        liftOne = (DcMotorEx) hwMap.get(DcMotor.class, "liftOne");
        liftTwo = (DcMotorEx) hwMap.get(DcMotor.class, "liftTwo");

        //---------------------------------------------------------------------------
        //endregion

        //region
        //------------------------===Servos===------------------------

        clawServo1 = hwMap.get(Servo.class, "clawServo1");
        clawServo2 = hwMap.get(Servo.class, "clawServo2");
        foundationLock = hwMap.get(Servo.class, "foundationLock");
        redAutoClawJoint1 = hwMap.get(Servo.class, "redAutoClawJoint1");
        redAutoClawJoint2 = hwMap.get(Servo.class, "redAutoClawJoint2");
        redAutoClawJoint3 = hwMap.get(Servo.class, "redAutoClawJoint3");
        transferLock = hwMap.get(Servo.class, "transferLock");
        transferHorn = hwMap.get(Servo.class, "transferHorn");
        clawInit = hwMap.get(Servo.class, "clawInit");
        innerTransfer = hwMap.get(Servo.class, "innerTransfer");
        intakeInit = hwMap.get(Servo.class, "intakeInit");
        //parkingServo = hwMap.get(Servo.class, "parkingServo");  //@TODO Configure intakeInit on the robot
        liftOdometry = hwMap.get(Servo.class, "liftOdometry");

        //---------------------------------------------------------------------------
        //endregion

        liftReset = hwMap.get(DigitalChannel.class, "liftReset");
        //intakeDetect = hwMap.get(DigitalChannel.class, "intakeDetect");   //@TODO Configure intakeDetect on the robot
        foundationDetectLeft = hwMap.get(DigitalChannel.class, "foundationDetectLeft");
        foundationDetectRight = hwMap.get(DigitalChannel.class, "foundationDetectRight");

        //---------------------------------------------------------------------------

        gyro = hwMap.get(BNO055IMU.class, "imu");
        imu = (IntegratingGyroscope) gyro;

        this.hardwareMap = hwMap;
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        gyro.initialize(parameters);
    }
}