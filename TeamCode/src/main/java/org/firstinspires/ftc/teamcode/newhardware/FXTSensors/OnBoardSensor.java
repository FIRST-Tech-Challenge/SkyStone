package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.hardware.Sensor;

/**
 * Created by FIXIT on 15-09-10.
 */
public class OnBoardSensor {

    private OnBoardSensorManager osm;
    private int sensorType;

    /**
     * Creates a new MySensor and registers the listener to be the sensor manager
     * @param sensorType the type of sensor. Must be standard sensor types
     * @param sensorManager the sensor manager that updates the storedValues
     */
    public OnBoardSensor (int sensorType, OnBoardSensorManager sensorManager) {
        this.sensorType = sensorType;
        sensorManager.addSensor(sensorType);
        this.osm = sensorManager;
    }//MySensor

    /**
     * Creates a new MySensor and registers the listener to be the sensor manager
     * @param sensorType the type of sensor. Must be standard sensor types
     * @param name the name of the sensor
     * @param sensorManager the sensor manager that updates the storedValues
     */
    public OnBoardSensor (int sensorType, String name, OnBoardSensorManager sensorManager) {
        this.sensorType = sensorType;
        sensorManager.addSensor(sensorType);
        this.osm = sensorManager;
    }//MySensor

    /**
     * method to get the type of the sensor in string form
     * @return The type of the sensor in a string
     */
    public String getTypeString() {
        switch (sensorType){
            case Sensor.TYPE_ACCELEROMETER: return "Accelerometer";
            case Sensor.TYPE_MAGNETIC_FIELD: return "Magnetometer";
            case Sensor.TYPE_ORIENTATION: return "Orientation";
            case Sensor.TYPE_GRAVITY: return "Gravity";
            case Sensor.TYPE_LIGHT: return "Light";
            case Sensor.TYPE_TEMPERATURE: return "Temperature";
            case Sensor.TYPE_PROXIMITY: return "Proximity";
            case Sensor.TYPE_LINEAR_ACCELERATION: return "Linear Acceleration";
            case Sensor.TYPE_ROTATION_VECTOR: return "Rotation Vector";
            default: return "Gyroscope";
        }//switch
    }//getName

    /**
     * Get the type of the sensor
     * @return the sensor type.
     */
    public int type() {
        return sensorType;
    }//type

    /**
     * Get the current storedValues of the sensor
     * @return the current storedValues
     */
    public double[] getValues() {
        return osm.getValues(this.sensorType);
    }//getValues

}