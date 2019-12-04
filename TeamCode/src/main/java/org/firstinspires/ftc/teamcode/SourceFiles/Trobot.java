package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Trobot {
    HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Component component;

    public ElapsedTime runtime;

    public Trobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        component = new Component(hardwareMap);

        runtime = new ElapsedTime();
    }
}