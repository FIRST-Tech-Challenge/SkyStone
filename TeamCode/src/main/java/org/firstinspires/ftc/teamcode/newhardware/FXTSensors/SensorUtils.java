package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.hardware.SensorManager;

/**
 * Created by FIXIT on 15-09-22.
 */
public class SensorUtils {

    /**
     * Get the pitch yaw and roll of the phone based on the magnetometer and accelerometer
     * @param accel the onboard accelerometer. Must be of type ACCEL.
     * @param mag the onboard magnetometer. Must be of type MAG.
     * @return a float array containing rotation around each axis in the form of Euler Angles
     */
    public static float[] getAllAngles (float[] accel, float[] mag) {
        float[] rotMatrix = new float[9];
        float[] incline = new float[9];
        float[] orientMatrix = new float[3];

        SensorManager.getInclination(incline);
        SensorManager.getRotationMatrix(rotMatrix, incline, accel, mag);
        SensorManager.getOrientation(rotMatrix, orientMatrix);

        for (int i = 0; i < orientMatrix.length; i++) {
            if (orientMatrix[i] < 0) {
                orientMatrix[i] += 2 * Math.PI;
            }//if
        }//for

        return orientMatrix;
    }

    public static float[] getAllAngles (double[] accelDouble, double[] magDouble) {
        float[] rotMatrix = new float[9];
        float[] incline = new float[9];
        float[] orientMatrix = new float[3];

        float[] accel = {(float) accelDouble[0],
                (float) accelDouble[1],
                (float) accelDouble[2]};

        float[] mag = {(float) magDouble[0],
                (float) magDouble[1],
                (float) magDouble[2]};

        SensorManager.getInclination(incline);
        SensorManager.getRotationMatrix(rotMatrix, incline, accel, mag);
        SensorManager.getOrientation(rotMatrix, orientMatrix);

        for (int i = 0; i < orientMatrix.length; i++) {
            if (orientMatrix[i] < 0) {
                orientMatrix[i] += 2 * Math.PI;
            }//if
        }//for

        return orientMatrix;
    }

    public static float[] getAllAngles (OnBoardSensor accelSensor, OnBoardSensor magSensor) {
        float[] rotMatrix = new float[9];
        float[] incline = new float[9];
        float[] orientMatrix = new float[3];

        double[] accelDouble = accelSensor.getValues();
        double[] magDouble = magSensor.getValues();

        float[] accel = {(float) accelDouble[0], (float) accelDouble[1], (float) accelDouble[2]};

        float[] mag = {(float) magDouble[0], (float) magDouble[1], (float) magDouble[2]};

        if (accel == null || mag == null) {
            throw new RuntimeException("Sensors used by getAllAngles() must be an accelerometer and a magnetometer");
        }

        SensorManager.getInclination(incline);
        SensorManager.getRotationMatrix(rotMatrix, incline, accel, mag);
        SensorManager.getOrientation(rotMatrix, orientMatrix);

        for (int i = 0; i < orientMatrix.length; i++) {
            if (orientMatrix[i] < 0) {
                orientMatrix[i] += 2 * Math.PI;
            }//if
        }//for

        return orientMatrix;
    }

    public static double lowPassFilter (double alpha, double newValue, double oldValue) {
        return oldValue + alpha * (newValue - oldValue);
    }

    public static double avg(double firstValue, double secondValue) {
        return (firstValue + secondValue) / 2;
    }

}
