package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

/**
 * Created by FIXIT on 2017-08-27.
 */

public class RGBTest extends TeleOpMode {

    ColorSensor col;

    @Override
    public void initialize() {
        col = RC.h.colorSensor.get("color");
    }

    @Override
    public void loopOpMode() {
        RC.t.addData("Colour Data", "Red: " + col.red() + ", Blue: " + col.blue() + ", Green: " + col.green());
    }
}
