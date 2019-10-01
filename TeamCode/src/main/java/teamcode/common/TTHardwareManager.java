package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTHardwareManager {

    private static final String[] COMPONENT_NAMES = {"FrontLeftDrive", "FrontRightDrive",
            "BackLeftDrive", "BackRightDrive", "ArmElbow", "ArmLift", "ArmLiftSensor", "intakeLeft", "intakeRight"};

    private final DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private final DcMotor armElbow, armLift;
    private final DistanceSensor armLiftSensor;
    private final DcMotor intakeLeft, intakeRight;

    public TTHardwareManager(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[0]);
        frontRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[1]);
        backLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[2]);
        backRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[3]);

        armElbow = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[4]);
        armLift = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[5]);
        armLiftSensor = hardwareMap.get(DistanceSensor.class, COMPONENT_NAMES[6]);
        intakeLeft = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[7]);
        intakeRight = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[8]);
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

    public DcMotor getIntakeLeft() {
        return intakeLeft;
    }

    public DcMotor getIntakeRight() {
        return intakeRight;
    }

}
