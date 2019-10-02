package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

public class TTHardwareManager {

    private static final String[] COMPONENT_NAMES = {"FrontLeftDrive", "FrontRightDrive",
            "BackLeftDrive", "BackRightDrive", "IntakeLeft", "IntakeRight"};

    private TTDriveSystem driveSystem;
    private TTIntake intake;

    public TTHardwareManager(HardwareMap hardwareMap, TTHardwareRestriction... hardwareRestrictions) {
        List<TTHardwareRestriction> hardwareRestrictionList = Arrays.asList(hardwareRestrictions);
        if (!hardwareRestrictionList.contains(TTHardwareRestriction.DISABLE_DRIVE)) {
            DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[0]);
            DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[1]);
            DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[2]);
            DcMotor backRightDrive = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[3]);
            this.driveSystem = new TTDriveSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        }
        if (!hardwareRestrictionList.contains(TTHardwareRestriction.DISABLE_INTAKE)) {
            DcMotor intakeLeft = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[4]);
            DcMotor intakeRight = hardwareMap.get(DcMotor.class, COMPONENT_NAMES[5]);
            this.intake = new TTIntake(intakeLeft, intakeRight);
        }
    }

    public TTDriveSystem getDriveSystem() {
        return driveSystem;
    }

    public TTIntake getIntake() {
        return intake;
    }

    public enum TTHardwareRestriction {
        DISABLE_DRIVE, DISABLE_INTAKE;
    }

}
