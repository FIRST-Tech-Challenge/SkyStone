package org.firstinspires.ftc.teamcode.All;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMap {
    public DcMotorEx backLeft, backRight, frontLeft, frontRight, linearSlider, firstJoint, secondJoint, intakeJoint;
    public CRServo intake;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public AnalogInput leftForward, rightForward, sideways;

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        backLeft = (DcMotorEx)hwMap.get(DcMotor.class, "backLeft");
        backRight = (DcMotorEx)hwMap.get(DcMotor.class, "backRight");
        frontLeft = (DcMotorEx)hwMap.get(DcMotor.class,"frontLeft");
        frontRight = (DcMotorEx)hwMap.get(DcMotor.class,"frontRight");

        leftForward = (AnalogInput)hwMap.get(AnalogInput.class, "leftForward");
        rightForward = (AnalogInput)hwMap.get(AnalogInput.class, "rightForward");
        sideways = (AnalogInput)hwMap.get(AnalogInput.class, "sideways");
        gyro = hwMap.get(BNO055IMU.class, "imu");
        //imu = (IntegratingGyroscope)gyro;


        //firstJointVirtualServo = new MotorServo(firstJoint, MotorServo.MotorConfiguration.firstJoint);
        //secondJointVirtualServo = new MotorServo(secondJoint, MotorServo.MotorConfiguration.secondJoint);

        this.hardwareMap = hwMap;
    }

    public void gyroInit(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        gyro.initialize(parameters);
    }

    public void resetEncoders() throws InterruptedException{
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(150);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static String TAG = "Robot_Status:";
}
