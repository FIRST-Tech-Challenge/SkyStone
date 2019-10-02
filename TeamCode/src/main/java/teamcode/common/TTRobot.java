package teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class TTRobot {

    private final TTHardwareManager hardwareManager;

    public TTRobot(HardwareMap hardwareMap, TTHardwareManager.TTHardwareRestriction... hardwareRestrictions) {
        hardwareManager = new TTHardwareManager(hardwareMap, hardwareRestrictions);
    }

    public TTHardwareManager getHardwareManager() {
        return hardwareManager;
    }

}
