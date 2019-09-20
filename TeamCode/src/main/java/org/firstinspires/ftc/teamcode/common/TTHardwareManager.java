package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTHardwareManager {

    private static final String[] COMPONENT_NAMES = {"FrontLeftDrive", "FrontRightDrive",
            "BackLeftDrive", "BackRightDrive", "ArmElbow", "ArmLift", "ArmLiftSensor", "ArmIntake", "Claw"};

    private final DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private final DcMotor armElbow, armLift, armIntake;
    private final DistanceSensor armLiftSensor;
    private final Servo claw;

    public TTHardwareManager(HardwareMap hardwareMap, TTHardwareRestriction hardwareRestriction) {
        if (hardwareRestriction == TTHardwareRestriction.ARM_ONLY) {
            frontLeftDrive = null;
            frontRightDrive = null;
            backLeftDrive = null;
            backRightDrive = null;
        } else {
            frontLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[0]);
            frontRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[1]);
            backLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[2]);
            backRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[3]);
        }

        if (hardwareRestriction == TTHardwareRestriction.DRIVE_SYSTEM_ONLY) {
            armElbow = null;
            armLift = null;
            armLiftSensor = null;
            armIntake = null;
            claw = null;
        } else {
            armElbow = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[4]);
            armLift = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[5]);
            armLiftSensor = hardwareMap.get(DistanceSensor.class, COMPONENT_NAMES[6]);
            armIntake = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[7]);
            claw = hardwareMap.get(Servo.class, COMPONENT_NAMES[8]);
        }
    }

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getBackLeftDrive() {
        return backLeftDrive;
    }

    public DcMotor getBackRightDrive() {
        return backRightDrive;
    }

    public DcMotor getArmElbow() {
        return armElbow;
    }

    public DcMotor getArmLift() {
        return armLift;
    }

    public DistanceSensor getArmLiftSensor() {
        return armLiftSensor;
    }

    public DcMotor getArmIntake(){
        return armIntake;
    }

    public Servo getClaw(){
        return claw;
    }

    public enum TTHardwareRestriction {
        NONE, ARM_ONLY, DRIVE_SYSTEM_ONLY
    }

}
