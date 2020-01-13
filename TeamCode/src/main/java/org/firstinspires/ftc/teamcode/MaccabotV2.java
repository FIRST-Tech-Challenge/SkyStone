package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MaccabotV2 {

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    MaccaDrive drive;

    public MaccabotV2(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }



}
