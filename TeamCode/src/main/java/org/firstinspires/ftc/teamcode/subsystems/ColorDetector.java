package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ColorDetector {

    public ColorSensor color_sensor;

    public void ColorDetector(HardwareMap hardwareMap) {

        color_sensor = hardwareMap.colorSensor.get("color");
    }

   public void setMode(boolean mode){

        color_sensor.enableLed(mode);
    }

    public boolean isRed() {
        if (color_sensor.red()>127&&color_sensor.green()<127&&color_sensor.blue()<127) {
            return true;
        }
        return false;
    }

    public boolean isBlue() {
        if (color_sensor.blue()>127&&color_sensor.green()<127&&color_sensor.red()<127) {
            return true;
        }
        return false;
    }

    public boolean isYellow() {
        if (color_sensor.red()>127&&color_sensor.green()>127&&color_sensor.blue()<127) {
            return true;
        }
        return false;
    }

}
