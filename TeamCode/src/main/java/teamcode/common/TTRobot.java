package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTRobot {

    //private TTDriveSystem driveSystem;
    //private TTVision vision;
    private TTArm arm;

    public TTRobot(HardwareMap hardwareMap) {
        //driveSystem = new TTDriveSystem(hardwareMap);
        //vision = new TTVision(hardwareMap);
        arm = new TTArm(hardwareMap);
    }

//    public TTDriveSystem getDriveSystem() {
//        return driveSystem;
//    }
//
//    public TTVision getVision() {
//        return vision;
//    }

    public TTArm getArm() { return arm; }

}
