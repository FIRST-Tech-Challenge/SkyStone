package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTRobot {

    private TTDriveSystem driveSystem;
    private TTVision vision;

    public TTRobot(HardwareMap hardwareMap) {
        driveSystem = new TTDriveSystem(hardwareMap);
        vision = new TTVision(hardwareMap);
    }

    public TTDriveSystem getDriveSystem() {
        return driveSystem;
    }

    public TTVision getVision() {
        return vision;
    }

}
