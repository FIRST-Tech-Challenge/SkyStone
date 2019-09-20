package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTRobot {

    private final TTHardwareManager hardwareManager;
    private final TTDriveSystem driveSystem;
    private final TTArm arm;

    public TTRobot(HardwareMap hardwareMap, TTHardwareManager.TTHardwareRestriction hardwareRestriction) {
        hardwareManager = new TTHardwareManager(hardwareMap, hardwareRestriction);

        if (hardwareRestriction == TTHardwareManager.TTHardwareRestriction.ARM_ONLY) {
            driveSystem = null;
        } else {
            DcMotor frontLeftDrive = hardwareManager.getFrontLeftDrive();
            DcMotor frontRightDrive = hardwareManager.getFrontRightDrive();
            DcMotor backLeftDrive = hardwareManager.getBackLeftDrive();
            DcMotor backRightDrive = hardwareManager.getBackRightDrive();
            driveSystem = new TTDriveSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        }

        if (hardwareRestriction == TTHardwareManager.TTHardwareRestriction.DRIVE_SYSTEM_ONLY) {
            arm = null;
        } else {
            DcMotor armElbow = hardwareManager.getArmElbow();
            DistanceSensor armLiftSensor = hardwareManager.getArmLiftSensor();
            DcMotor armLift = hardwareManager.getArmLift();
            DcMotor armIntake = hardwareManager.getArmIntake();
            Servo claw = hardwareManager.getClaw();
            arm = new TTArm(armLift, armLiftSensor, armElbow, armIntake, claw);
        }
    }

    public TTHardwareManager getHardwareManager() {
        return hardwareManager;
    }

    public TTDriveSystem getDriveSystem() {
        return driveSystem;
    }

    public TTArm getArm() {
        return arm;
    }

}
