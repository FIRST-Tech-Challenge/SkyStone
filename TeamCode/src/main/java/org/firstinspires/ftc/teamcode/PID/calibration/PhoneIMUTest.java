package org.firstinspires.ftc.teamcode.PID.calibration;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class PhoneIMUTest extends LinearOpMode {
    private static String TAG = "PhoneIMUTest";

    private SensorUpdates sensorUpate;
    private SensorManager sensors = FtcRobotControllerActivity.sensors;


    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();

        List<Sensor> sensorList = sensors.getSensorList(Sensor.TYPE_ALL);

        String sensorInfo = "";
        for (Sensor s : sensorList){
            sensorInfo= sensorInfo + s.getName()+ "\n";
        }
        RobotLog.dd(TAG, "sensor list: " + sensorInfo);

        Sensor accelerometer = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magneticField = sensors.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor orientation = sensors.getDefaultSensor(Sensor.TYPE_ORIENTATION);

        if (accelerometer != null) {
            sensors.registerListener(sensorUpate, accelerometer, SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_FASTEST);
        }

        if (magneticField != null) {
            sensors.registerListener(sensorUpate, magneticField, SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (orientation != null) {
            sensors.registerListener(sensorUpate, orientation, SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_FASTEST);
        }
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

        }
        sensors.unregisterListener(sensorUpate);
    }
}

class SensorUpdates implements SensorEventListener {
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];

    private final float[] rotationMatrix = new float[9];
    private final float[] orientationAngles = new float[3];
    private long lastUpdate = 0;
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (lastUpdate == 0) {
            lastUpdate = event.timestamp;
        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, accelerometerReading,
                    0, accelerometerReading.length);
        }
        else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, magnetometerReading,
                    0, magnetometerReading.length);
        }
        else if (event.sensor.getType() == Sensor.TYPE_ORIENTATION) {
            float azimuth = event.values[0];
            RobotLog.dd("SensorUpdates", Float.toString(azimuth));
        }
        final float[] rotationMatrix = new float[9];
        SensorManager.getRotationMatrix(rotationMatrix, null,
                accelerometerReading, magnetometerReading);

        float x = accelerometerReading[0];
        float y = accelerometerReading[1];
        float z = accelerometerReading[2];

        float accelationSquareRoot = (x * x + y * y + z * z)
                / (SensorManager.GRAVITY_EARTH * SensorManager.GRAVITY_EARTH);
        long actualTime = event.timestamp;
        if (accelationSquareRoot >= 2) //
        {
            if (actualTime - lastUpdate < 200) {
                return;
            }
            RobotLog.dd("SensorUpdates", "something happened");
            lastUpdate = actualTime;
        }

// Express the updated rotation matrix as three orientation angles.
        final float[] orientationAngles = new float[3];
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        RobotLog.dd("SensorUpdates", "onAccuracyChanged");
    }
}