package org.firstinspires.ftc.teamcode.subsystems.color_sensor;

/**
 * Created by Sarthak on 9/7/2018.
 */
public interface IColorSensor {

    /**
     * Returns the color sensors red value
     * @return the red value as an integer
     */
    public int red();
    /**
     * Returns the color sensors green value
     * @return the green value as an integer
     */
    public int green();
    /**
     * Returns the color sensors blue value
     * @return the blue value as an integer
     */
    public int blue();
    /**
     * Returns the color sensors hue value
     * @return the hue value as an integer
     */
    public int getHue();
    /**
     * Returns the color sensors alpha value
     * @return the alpha value as an integer
     */
    public int alpha();

    /**
     * Collects the Hue, Saturation, and Value from the sensor
     * @return the HSV value in a float array
     */
    public float[] getHSV();

}
