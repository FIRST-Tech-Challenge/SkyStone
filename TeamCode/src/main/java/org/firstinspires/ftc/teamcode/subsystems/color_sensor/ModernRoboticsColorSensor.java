package org.firstinspires.ftc.teamcode.subsystems.color_sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Sarthak on 10/16/2017.
 */

public class ModernRoboticsColorSensor implements IColorSensor {
    //Create color sensor
    ColorSensor color;

    /**
     * Constructor for Modern Robotics Color Sensor
     * @param color color sensor to make readings
     */
    public ModernRoboticsColorSensor(ColorSensor color){
        this.color = color;
    }

    /**
     * Returns the red value from the sensor reading
     * @return the red value as an integer
     */
    @Override
    public int red() {
        return color.red();
    }

    /**
     * Returns the green value from the sensor reading
     * @return the green value as an integer
     */
    @Override
    public int green() {
        return color.green();
    }

    /**
     * Returns the blue value from the sensor reading
     * @return the blue value as an integer
     */
    @Override
    public int blue() {
        return color.blue();
    }

    /**
     * Returns the hue value from the sensor reading
     * @return the hue value as an integer
     */
    @Override
    public int getHue() {
        return color.argb();
    }

    /**
     * Returns the alpha value from the sensor reading
     * @return the alpha value as an integer
     */
    @Override
    public int alpha() {
        return color.alpha();
    }

    /**
     * Collects the HSV (Hue, Saturation, Value) and returns all three values in the form of a float array
     * @return the HSV value within a float array
     */
    @Override
    public float[] getHSV() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(this.red(), this.green(), this.blue(), hsv);
        return hsv;
    }
}
