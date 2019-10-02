package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class StoneManipulator {

    private static StoneManipulator instance = null;

    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }

    private StoneManipulator() {}

    public void init(HardwareMap hardwareMap) {

    }
}
