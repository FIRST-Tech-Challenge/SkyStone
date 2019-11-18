package org.firstinspires.ftc.teamcode.Tensorflow;


import android.hardware.Camera;

import java.util.ArrayList;

public class TFODCalc {
    private static float FOCAL_LENGTH = 1;    //in mm
    private static double SENSOR_HEIGHT = 1.0;    //in mm
    private static Camera camera;

    public static void getPhoneCamConstants(){
        camera = Camera.open();
        Camera.Parameters params = camera.getParameters();
        FOCAL_LENGTH = params.getFocalLength();
        float verticalViewAngle = params.getVerticalViewAngle();
        camera.release();
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalViewAngle / 2)) * 2 * FOCAL_LENGTH;  //tan(angle/2) * 2 * focalLength (in degrees)
    }

    public static float getFocalLength(){ return FOCAL_LENGTH; }

    public static double getSensorHeight(){ return SENSOR_HEIGHT; }

    public static void setFocalLength(float focalLength){ FOCAL_LENGTH = focalLength; }

    public static void setSensorHeight(double sensorHeight){ SENSOR_HEIGHT = sensorHeight; }

    public static double getDistanceToObj(double objHeightmm, double imgHeightpx, double objHeightpx){
        double dist = (FOCAL_LENGTH * objHeightmm * imgHeightpx) / (objHeightpx * SENSOR_HEIGHT) / 25.4;   //in inches (mm / 25.4)
        return dist;
    }

    public static ArrayList<Double> getAngleOfStone(double objWidthPx, double distance){
        double xIntercept = 307.369;
        double estimated0DegreeWidth = 800.512823871366 * Math.pow(Math.E, -0.0476285053327913 * distance) - 15;
        double xIntOffset = xIntercept - estimated0DegreeWidth;    //Calculates width if 0 degree angle (Face facing camera)
        double theta = -0.00402486517332459 * Math.pow((objWidthPx + xIntOffset), 2) + 1.34744719385825 *
                (objWidthPx + xIntOffset) - 33.9109714390624;  //Calculates angle
        ArrayList<Double> output = new ArrayList<Double>(){{ add(theta);
            add(Math.round((estimated0DegreeWidth - 197.369) * 1000.0) / 1000.0);
            add(Math.round((estimated0DegreeWidth + 1.631) * 1000.0) / 1000.0);
            add(Math.round(estimated0DegreeWidth * 1000.0) / 1000.0); }};
        return output;
    }
}