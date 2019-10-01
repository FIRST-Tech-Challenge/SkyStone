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
    private final TTIntake intake;

    public TTRobot(HardwareMap hardwareMap) {
        hardwareManager = new TTHardwareManager(hardwareMap);

        DcMotor frontLeftDrive = hardwareManager.getFrontLeftDrive();
        DcMotor frontRightDrive = hardwareManager.getFrontRightDrive();
        DcMotor backLeftDrive = hardwareManager.getBackLeftDrive();
        DcMotor backRightDrive = hardwareManager.getBackRightDrive();
        driveSystem = new TTDriveSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        DcMotor armElbow = hardwareManager.getArmElbow();
        DistanceSensor armLiftSensor = hardwareManager.getArmLiftSensor();
        DcMotor armLift = hardwareManager.getArmLift();
        arm = new TTArm(armLift, armLiftSensor, armElbow);

        DcMotor intakeLeft = hardwareManager.getIntakeLeft();
        DcMotor intakeRight = hardwareManager.getIntakeRight();
        intake = new TTIntake(intakeLeft, intakeRight);
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
