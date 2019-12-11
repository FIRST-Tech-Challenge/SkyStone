package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.hardware.hardwareutils.HardwareManager;

public class AutonColorSensor extends OpMode {
    public HardwareManager hardware;

    public ColorSensor colorsensor;
    public AutoCommands autoCommands;

    public void init() {
        hardware = new HardwareManager(hardwareMap);
        autoCommands = new AutoCommands(hardware, telemetry);
        colorsensor = hardware.colorSensor;
    }

    public void loop() {
        if (colorsensor.red() < 20){
            autoCommands.driveForward(0.75);
        } else {
            autoCommands.driveForward(0.0);
        }
    }
}
