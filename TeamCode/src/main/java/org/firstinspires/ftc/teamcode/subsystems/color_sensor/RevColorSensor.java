package org.firstinspires.ftc.teamcode.subsystems.color_sensor;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

/**
 * Created by Sarthak on 9/7/2018.
 */
public class RevColorSensor implements IColorSensor {

    //Create color sensor
    private LynxI2cColorRangeSensor color;

    /**
     * Constructor for REV Color Sensor
     * @param color sensor to make readings
     */
    public RevColorSensor(LynxI2cColorRangeSensor color){
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
     * @return the hue as an integer
     */
    @Override
    public int getHue() {
        return color.argb();
    }

    /**
     * Collects the HSV (Hue, Saturation, Value) and returns all three values in the form of a float array
     * @return the HSV value within a float array
     */
    public float[] getHSV(){
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(this.red(), this.green(), this.blue(), hsv);
        return hsv;
    }

    /**
     * Reads the alpha value from the color sensor
     * @return the alpha value as an integer
     */
    @Override
    public int alpha() {
        return color.alpha();
    }
}
