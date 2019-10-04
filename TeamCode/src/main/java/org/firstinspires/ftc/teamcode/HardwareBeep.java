//Declare package
package org.firstinspires.ftc.teamcode;

//Import Hardware

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our Hardware Map that contains all the motors, servos, and sensors that we use on the
 * robot. We pull this Hardware Map in all the programs we use a part of the robot. In this program
 * we intialize the encoders on the motors we want to call the encoder for.
 */
public class HardwareBeep {

    // Set Public OpMode Members
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor lift = null;
    public DcMotor arm = null;
    public DcMotor armExtrusion = null;
    public DcMotor intake = null;
    public CRServo latch = null;
    public Servo basket = null;
    public BNO055IMU imu = null;
    public ModernRoboticsI2cColorSensor colorSensor = null;
    public SensorMB1242 rightSonic = null;
    public SensorMB1242 leftSonic = null;
    public DigitalChannel touchSensor = null;

    // Set local OpMode Members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public HardwareBeep() {
    }

    /**
     * Initializes standard hardware interfaces
     *
     * @param ahwMap A reference to Hardware Map
     */
    public void init(HardwareMap ahwMap) {

        // Telemetry Switches
        boolean GRID_NAV_TELEMETRY_ON = true;

        // Save Reference To Hardware Map
        hwMap = ahwMap;

        // Define Motors, Servos, and Sensors
        leftFront = hwMap.get(DcMotor.class, "left_front");
        leftBack = hwMap.get(DcMotor.class, "left_back");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        rightBack = hwMap.get(DcMotor.class, "right_back");
        lift = hwMap.get(DcMotor.class, "lift");
        latch = hwMap.get(CRServo.class, "latch");
        arm = hwMap.get(DcMotor.class, "arm");
        armExtrusion = hwMap.get(DcMotor.class, "arm_extrusion");
        intake = hwMap.get(DcMotor.class, "intake");
        basket = hwMap.get(Servo.class, "basket");
        imu = hwMap.get(BNO055IMU.class, "imu");
        colorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "color_Sensor");
        leftSonic = hwMap.get(SensorMB1242.class, "left_sonic");
        leftSonic.changeI2cAddress(125);
        rightSonic = hwMap.get(SensorMB1242.class, "right_sonic");
        rightSonic.changeI2cAddress(126);
        touchSensor = hwMap.get(DigitalChannel.class, "sensor_digital");

        // Set Motor and Servo Direction
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        armExtrusion.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        latch.setDirection(CRServo.Direction.FORWARD);

        // Set Motor to Zero Power Behavior
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtrusion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servos to Zero Power
        latch.setPower(0);

        // Set Motors to Run Without Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Motors to Run Using Encoders
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtrusion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize IMU
        imu.initialize(parameters);
    }
}
