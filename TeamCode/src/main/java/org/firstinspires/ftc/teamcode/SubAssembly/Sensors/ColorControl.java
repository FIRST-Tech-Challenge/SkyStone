package org.firstinspires.ftc.teamcode.SubAssembly.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorControl {/* Declare private class object */
    private LinearOpMode opmode = null; /* local copy of HardwareMap object from opmode class */

    //initializing motors
    private ColorSensor sensorColor;
    int blueV;
    int redV;
    public int COLOR_THRESHOLD = 125;
    boolean blue;
    boolean red;

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
        sensorColor = hwMap.get(ColorSensor.class, "Color");
    }

    public void Telemetry() {
        opmode.telemetry.addData("Blue value: ", sensorColor.blue());
        opmode.telemetry.addData("Red value: ", sensorColor.red());
    }

    public int getRed() {
        return sensorColor.red();
    }

    public int getBlue() {
        return sensorColor.blue();
    }
}
