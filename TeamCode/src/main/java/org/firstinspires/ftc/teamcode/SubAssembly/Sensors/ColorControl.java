package org.firstinspires.ftc.teamcode.SubAssembly.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */
    private ColorSensor sensorColor;
    private int COLOR_THRESHOLD = 8000;

    /* Subassembly constructor */
    public ColorControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opMode.telemetry.addLine("Color Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        /* Map hardware devices */
        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
    }

    public void Telemetry() {
        opmode.telemetry.addData("Blue value: ", getBlue());
        opmode.telemetry.addData("Red value: ", getRed());
        if (isBlue())
            opmode.telemetry.addLine("BLUE");
        if (isRed())
            opmode.telemetry.addLine("RED");
    }

    public int getRed() {
        return sensorColor.red();
    }

    public int getBlue() {
        return sensorColor.blue();
    }

    public boolean isRed() {
        return getRed() >= COLOR_THRESHOLD;
    }

    public boolean isBlue() {
        return getBlue() >= COLOR_THRESHOLD;
    }

}
