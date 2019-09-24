package org.firstinspires.ftc.teamcode.subsystems.range_sensor;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sarthak on 9/7/2018.
 */
public class RevColorDistanceSensor implements IRangeSensor{

    //Create sensor hardware
    private LynxI2cColorRangeSensor ultrasonic;

    /**
     * Constructor for Rev Color/Range sensor
     * @param ultrasonic Rev Range Sensor hardware
     */
    public RevColorDistanceSensor(LynxI2cColorRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    /**
     * Reads the sensor and returns the distance for the target in centimeters
     * @return the distance in centimeters, as a double
     */
    @Override
    public double cmDistance() {
        return ultrasonic.getDistance(DistanceUnit.CM);
    }

}
