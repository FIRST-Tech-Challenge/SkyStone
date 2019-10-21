package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.teamcode.RC;

import java.util.HashMap;

/**
 * Created by FIXIT on 15-09-10.
 */
public class OnBoardSensorManager implements SensorEventListener {

    /**
     * Onboard Accelerometer
     */
    public final static int ACCEL = Sensor.TYPE_ACCELEROMETER;

    /**
     * Onboard Magnetometer
     */
    public final static int MAG = Sensor.TYPE_MAGNETIC_FIELD;
    /**
     * Onboard orientation sensor
     * @deprecated
     */
    @Deprecated
    public final static int ORIENT = Sensor.TYPE_ORIENTATION;

    /**
     * Onboard Gyro sensor
     */
    public final static int GYRO = Sensor.TYPE_GYROSCOPE;

    /**
     * Onboard Gravity sensor.
     */
    public final static int GRAV = Sensor.TYPE_GRAVITY;

    /**
     * Onboard Light sensor
     */
    public final static int LIGHT = Sensor.TYPE_LIGHT;

    /**
     * Onboard Thermometer
     */
    public final static int TEMP = Sensor.TYPE_TEMPERATURE;

    /**
     * Onboard Proximity Sensor
     */
    public final static int PROX = Sensor.TYPE_PROXIMITY;

    /**
     * Onboard Linear Accelerometer
     */
    public final static int LINEAR_ACC = Sensor.TYPE_LINEAR_ACCELERATION;

    /**
     * Onboard Rotation Vector Sensor
     */
    public final static int ROTATION_VECTOR = Sensor.TYPE_ROTATION_VECTOR;

    /**
     * Underlying sensor manager to deal with the sensors
     */
    public SensorManager mSensorManager;

    /**
     * Map allowing individual sensors to access their storedValues
     */
    private HashMap<Integer, float[]> sensors = new HashMap<Integer, float[]>();

    /**CONSTRUCTORS**/

    public OnBoardSensorManager() {
        mSensorManager = (SensorManager) RC.c().getSystemService(Context.SENSOR_SERVICE);

    }

    /**
     * Adds a sensor to use
     * @param sensorType type of sensor to add
     */
    public void addSensor(int sensorType) {
        sensors.put(sensorType, new float[3]);

        mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(sensorType), SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void addSensor(int sensorType, int arrayLength) {
        sensors.put(sensorType, new float[arrayLength]);

        mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(sensorType), SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        sensors.put(event.sensor.getType(), event.values.clone());

        callOnSensorChange(event);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void callOnSensorChange (SensorEvent event) {
        //User-defined method
    }

    public double[] getValues(int sensorType) {
        float[] values = sensors.get(sensorType);
        return new double[] {values[0], values[1], values[2]};
    }
}
