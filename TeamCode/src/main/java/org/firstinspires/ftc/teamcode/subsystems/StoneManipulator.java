package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class StoneManipulator {

    private static StoneManipulator instance = null;

    public static StoneManipulator getInstance(HardwareMap hardwareMap) {
        return instance != null ? instance : (instance = new StoneManipulator(hardwareMap));
    }

    private StoneManipulator(HardwareMap hardwareMap) {

    }
}
