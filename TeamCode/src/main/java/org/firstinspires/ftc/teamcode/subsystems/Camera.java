package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Camera {
    private static Camera instance = null;

    public static Camera getInstance(HardwareMap hardwareMap) {
        return instance != null ? instance : (instance = new Camera(hardwareMap));
    }

    private Camera (HardwareMap hardwareMap) {

    }
}
