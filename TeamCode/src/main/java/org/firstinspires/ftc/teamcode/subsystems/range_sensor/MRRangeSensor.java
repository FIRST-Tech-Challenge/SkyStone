package org.firstinspires.ftc.teamcode.subsystems.range_sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import java.util.Arrays;

/**
 * Created by Sarthak on 9/7/2018.
 */

public class MRRangeSensor implements IRangeSensor {
    //Create ultrasonic
    private ModernRoboticsI2cRangeSensor ultrasonic;
    //Create instance variables
    private boolean firstRead = false;
    private double prevValue, currentValue;

    /**
     * Constructor for Modern Robotics Range Sensor
     * @param ultrasonic range sensor hardware
     */
    public MRRangeSensor(ModernRoboticsI2cRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    /**
     * Returns the distance read by the sensor
     * @return the distance in centimeters, as a double type
     */
    @Override
    public double cmDistance() {
        if(!firstRead){
            prevValue = ultrasonic.cmUltrasonic();
            firstRead = true;
        }else{
            currentValue = lowPass(ultrasonic.cmUltrasonic(), 0.75, prevValue);
            prevValue = currentValue;
        }
        return currentValue;
    }

    /**
     * Reads the optical distance from the sensor. Use for close-up targets
     * @return the optical distance as a double
     */
    public double opticalDistance() { return ultrasonic.cmOptical(); }

    /**
     * Sensor filter that calculates the median value of five sensor readings
     * @return the filtered ultrasonic value as an integer
     */
    private int smoothMedian(){
        double[] values = new double[5];
        for(int i = 0; i < values.length; i++){
            values[i] = ultrasonic.cmUltrasonic();
        }
        Arrays.sort(values);
        double median;
        if (values.length % 2 == 0) {
            median = ((double) values[values.length / 2] + (double) values[values.length / 2 - 1]) / 2;
        }
        else {
            median = (double) values[values.length / 2];
        }
        return (int) median;
    }

    /**
     * Low pass sensor filter to reduce the effect of spikes in the data
     * @param sensorReading current value the sensor returned
     * @param filterValue gain to multiply the reading by
     * @param smoothedValue previous filtered value from sensor
     * @return
     */
    private int lowPass(double sensorReading, double filterValue, double smoothedValue){
        if(filterValue >= 1){
            filterValue = 0.99;
        }else if(filterValue <= 0){
            filterValue = 0;
        }

        smoothedValue = (sensorReading * (1-filterValue)) + (smoothedValue*filterValue);
        return (int) smoothedValue;
    }
}
