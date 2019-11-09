package org.firstinspires.ftc.teamcode.Tensorflow;


import android.hardware.Camera;

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

    public static double getAngleOfObj(double objWidthPx, double distance){
        double xIntOffset = 492.841 - (786.7 * Math.pow(Math.E, -0.049 * distance));    //Calculates width if 0 degree angle (Face facing camera)
        xIntOffset = 0;
        double theta = Math.pow(5.54560717 * 10, -12) * Math.pow(objWidthPx + xIntOffset, 6) - Math.pow(1.096973892027 * 10, -8) *
                Math.pow(objWidthPx + xIntOffset, 5) + Math.pow(8.74991044895645 * 10, -6) * Math.pow(objWidthPx + xIntOffset, 4) -
                0.00358997566920175 * Math.pow(objWidthPx + xIntOffset, 3) + 0.795649580979338 * Math.pow(objWidthPx + xIntOffset, 2) -
                90.0326365149991 * (objWidthPx + xIntOffset) + 4133.59786820855;  //Calculates angle
        return theta;
    }
}