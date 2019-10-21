package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;


public class Armstrong extends Robot {
    public DigitalChannel magnetSensor;
    private Motor lifter;
    private FXTServo marker;
    private FXTServo latch;
    public LynxEmbeddedIMU imu;
    private float GEAR_RATIO = 32/16;
    public float MOTOR_SPEED;
    public float MOTOR_SPEED_PAST;
    public float AVR_MOTOR_DIFF;
    public Motor CollectMotor;
    private FXTServo linear;
    private FXTServo rightWing;
    private FXTServo leftWing;
    private FXTCRServo leftCollectServo;
    private FXTCRServo rightCollectServo;

    public FXTAnalogUltrasonicSensor ultrasonic;



    //private long lift;
    public Armstrong() {
        super();
        lifter = new Motor("lifter");
        marker = new FXTServo("marker");
        latch = new FXTServo("latch");
     // new motors and servos
        CollectMotor = new Motor("CollectMotor");
        linear =  new FXTServo("linear");
        rightWing = new FXTServo("rightWing");
        leftWing = new FXTServo("leftWing");
        leftCollectServo = new FXTCRServo("leftCollectServo");
        rightCollectServo = new FXTCRServo("rightCollectServo");
        magnetSensor = RC.h.get(DigitalChannel.class, "sensor_digital");
        magnetSensor.setMode(DigitalChannel.Mode.INPUT);
        FXTAnalogUltrasonicSensor ultrasonic = new FXTAnalogUltrasonicSensor("ultrasonic");
        motorL.setMinimumSpeed(0.1);
        motorR.setMinimumSpeed(0.1);

        //Armstrong armstrong = new Armstrong();

        wheelDiameter = wheelDiameter * GEAR_RATIO;


        //lift = 8000;

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = (LynxEmbeddedIMU) RC.h.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        markUp();
        LeftWingStore();
        RightWingStore();


    }




    //setting direction for lifter
    public void lifterUp(){
        lifter.setPower(-1);
    }
    public void lifterDown(){ lifter.setPower(1); }
    public void lifterStop(){lifter.setPower(0); }

    //setting position for marker servo
    public void markUp() {marker.setPosition(1.0);}
    public void markDown() { marker.setPosition(0.45);}
    public void markWallDown(){
        RightSample();
        LeftSample();
        RC.l.sleep(1000);
        markDown();
        RC.l.sleep(1000);
        markUp();
        RightWingStore();
        LeftWingStore();
    }

    //setting latch
    public void unlatch() {latch.setPosition(0.1);}
    public void setLatch() {latch.setPosition(0.9);}






    public void collectServoLeftDown(){leftCollectServo.setPower(-0.7);}
    public void collectServoRightDown(){rightCollectServo.setPower(0.7);}

    public void collectServoLeftUp(){leftCollectServo.setPower(0.7);}
    public void collectServoRightUp(){rightCollectServo.setPower(-0.7);}

    public void collectServoRightStop() {rightCollectServo.setPower(0);}
    public void collectServoLeftStop() {leftCollectServo.setPower(0);}

    public void collectServoLeftSlow() {leftCollectServo.setPower(0.05);}
    public void collectServoRightSlow() {rightCollectServo.setPower(-0.05);}

    public void armup() {CollectMotor.setPower(0.2);}
    public void armupslow() {CollectMotor.setPower(0.1);}
    public void armdown() {CollectMotor.setPower(-0.2);}
    public void armdownslow() {CollectMotor.setPower(-0.1);}
    public void armstop() {CollectMotor.setPower(0);}

    //setting IMU
    public double getAngle() {return -imu.getAngularOrientation().firstAngle;}

    public void RightSample() {rightWing.setPosition(0.1);}
    public void LeftSample()  {leftWing.setPosition(0.96);}

    public void RightWingStore() {rightWing.setPosition(0.8);}
    public void LeftWingStore() {leftWing.setPosition(0.1);}

    public void wallPush(){linear.setPosition(0.3);}
    public void wallIn() {linear.setPosition(0.7);}
    //public void wallStop() {linear.setPosition(0);}

    public void MiddleSample() {
        LeftSample();
        RightSample();
        wallPush();
        RC.l.sleep(1000);
        RightWingStore();
        LeftWingStore();
        RC.l.sleep(1000);
        wallIn();
    }

    public void RightForwardSample(){
        //in here will go the min right program
    }

    //
//    public static state = magnetSensor.getState();

    //Set Wall-E position




}
