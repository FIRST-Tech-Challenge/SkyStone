package org.firstinspires.ftc.teamcode.hardware.hardwareutils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Contains all pieces of Hardware used on Robot. Only declare & initialize Hardware here.
 */
public class  HardwareManager {

    HardwareMap hardwareMap;

    // Drive train Motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightRearDrive;
    public DcMotor leftRearDrive;
    public ColorSensor colorSensor;

    public DcMotor rightIntakeMotor;
    public DcMotor leftIntakeMotor;

    // Elevator motors
    public DcMotor elevatorMotor;

    // Block Grabber
    public CRServo blockPanServo;

    // Latching Mechanism Servo
    public CRServo latch;

    // Intake Mechanism Servo
    public Servo boot;

    public HardwareManager(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initDriveTrain();
        initGrabber();
        initColorSensor();
        initIntake();
    }

    private void initDriveTrain() {
        // Set the motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, HardwareNames.leftFrontDrive);
        leftRearDrive = hardwareMap.get(DcMotor.class, HardwareNames.leftRearDrive);
        rightFrontDrive = hardwareMap.get(DcMotor.class, HardwareNames.rightFrontDrive);
        rightRearDrive = hardwareMap.get(DcMotor.class, HardwareNames.rightRearDrive);
        // Reverse right motors
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initGrabber() {
        elevatorMotor = hardwareMap.get(DcMotor.class, HardwareNames.elevatorMotor);
        latch = hardwareMap.get(CRServo.class, HardwareNames.latchingServo);
        blockPanServo = hardwareMap.get(CRServo.class, HardwareNames.blockPanServo);
    }

    private void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareNames.colorSensor);
    }

    private void initIntake() {
        rightIntakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.rightIntakeMotor);
        leftIntakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.leftIntakeMotor);
        boot = hardwareMap.get(Servo.class, HardwareNames.bootServo);
    }
}
