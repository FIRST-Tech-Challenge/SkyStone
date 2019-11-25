package org.firstinspires.ftc.teamcode.Tensorflow;


import android.hardware.Camera;

import java.util.ArrayList;

public class TFODCalc {
    private static float FOCAL_LENGTH = 1.0f;    //in mm
    private static double SENSOR_HEIGHT = 1.0;    //in mm
    private static Camera camera;
    private static ArrayList<Double> autoAdjustedOffset = new ArrayList<>();
    private static ArrayList<Double> distanceAway = new ArrayList<>();

    public static void init(){
        for(int i = 0; i < 100; i++)
            autoAdjustedOffset.add(0.0);
    }

    public static double getSavedOffset(int index){
        return autoAdjustedOffset.get(index);
    }

    public static void getPhoneCamConstants() {
        camera = Camera.open();
        Camera.Parameters params = camera.getParameters();
        FOCAL_LENGTH = params.getFocalLength();
        float verticalViewAngle = params.getVerticalViewAngle();
        camera.release();
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalViewAngle / 2)) * 2 * FOCAL_LENGTH;  //tan(angle/2) * 2 * focalLength (in degrees)
    }

    public static float getFocalLength(){ return FOCAL_LENGTH; }

    public static double getSensorHeight(){ return SENSOR_HEIGHT; }

    public static void setHardwareProperties(float focalLength, double sensorHeight){
        FOCAL_LENGTH = focalLength;
        SENSOR_HEIGHT = sensorHeight;
    }

    public static void setHardwareProperties(double verticalFOVAngle, float focalLength){
        FOCAL_LENGTH = focalLength;
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalFOVAngle / 2)) * 2 * FOCAL_LENGTH;
    }

    public static double getDistanceToObj(double objHeightmm, double imgHeightpx, double objHeightpx){
        double dist = (FOCAL_LENGTH * objHeightmm * imgHeightpx) / (objHeightpx * SENSOR_HEIGHT) / 25.4;   //in inches (mm / 25.4)
        return dist;
    }

    public static ArrayList<Double> getAngleOfStone(int objIndex, double objWidthPx, double distance){
        double xIntercept = 307.369;    //Largest X-Intercept of quadratic model equation
        double xAt60 = xIntercept - 197.369;    //Calculating Delta X at lower bounds of domain (x ~= 60°)
        double estimated0DegreeWidth = 800.512823871366 * Math.pow(Math.E, -0.0476285053327913 * distance) - 15;    //Estimating 0° width

        /**
         * Calculates "old" X-offset (X-int & other previously calculated offsets), the supposed width
         * of stone at 0° (stone facing camera). Algorithms will soon tune this value and adjust the model's domain
         * to better fit the received Tensorflow data.
         */

        double xIntOffset = xIntercept - estimated0DegreeWidth - autoAdjustedOffset.get(objIndex);  //The original x-offset value
        double newAutoAdjustedOffset = autoAdjustDomain(xIntercept, xAt60, xIntOffset, objWidthPx, objIndex); //Algorithm tunes the value of the offset, gets additional offset
        xIntOffset = xIntercept - estimated0DegreeWidth - autoAdjustedOffset.get(objIndex) - newAutoAdjustedOffset;    //Re-calculates the X-offset

        /**
         * The model quadratic equation used:
         * y = -0.00402486517332459(x + x-offset)^2 + 1.34744719385825(x + x-offset) - 33.9109714390624
         */

        double theta = -0.00402486517332459 * Math.pow((objWidthPx + xIntOffset), 2) + 1.34744719385825 *
                (objWidthPx + xIntOffset) - 33.9109714390624;  //Calculates angle

        autoAdjustedOffset.set(objIndex,
                autoAdjustedOffset.get(objIndex) + newAutoAdjustedOffset);   //The x-offset is saved under its index & will be reaccessed again and used

        /**
         * Output: ArrayList containing the following (in this order):
         * [Predicted Angle, Lower Bounds of model Domain, Upper Bounds of Model Domain (X-int),
         * Unprocessed predicted width at 0°, New predicted width at 0°]
         */

        ArrayList<Double> output = new ArrayList<>();
        output.add(theta);
        output.add(Math.round((estimated0DegreeWidth - 197.369 + newAutoAdjustedOffset) * 1000.0) / 1000.0);
        output.add(Math.round((estimated0DegreeWidth + newAutoAdjustedOffset) * 1000.0) / 1000.0);
        output.add(Math.round(estimated0DegreeWidth * 1000.0) / 1000.0);
        output.add(Math.round((estimated0DegreeWidth + newAutoAdjustedOffset +
                TFODCalc.autoAdjustedOffset.get(objIndex)) * 1000.0) / 1000.0);

        return output;
    }

    private static double autoAdjustDomain(double xInt, double xAt60, double xIntOffset,
                                         double objWidthPx, int objIndex){
        double autoAdjustOutput = 0.0;

        /**
         * Allows up to a certain degree of error. If data is past the acceptable error, then tuning algorithm changes
         * the X-offset of the model quadratic, thus better fitting the data. The algorithm finds the delta
         * between the predicted and actual x-intercepts by finding the difference in x values at a certain point.
         */

        if(xInt - objWidthPx <= xInt + xIntOffset - 207.369){  //Allows up to -5° of error (- 10)
            double allowedErrorOffset = xAt60 - (xInt + autoAdjustedOffset.get(objIndex) - 207.369);    //Calculates allowed error offset (- 10)
            autoAdjustOutput = getDomainOffset(xAt60 - allowedErrorOffset, xIntOffset, objWidthPx); //Gets new, additional offset
        } else if(xInt - objWidthPx >= xInt + xIntOffset + 10.611){   //Allows up to +10° of error (+ 8.611)
            autoAdjustOutput = getDomainOffset(xInt + 10.611, xIntOffset, objWidthPx);   //Calculates new, additional offset
        }
        return autoAdjustOutput;
    }

    private static double getDomainOffset(double anchorX, double xIntOffset, double objWidthPx){
        double newOffset = anchorX - objWidthPx;    //Actual offset based on current width of the skystone
        return xIntOffset - newOffset;  //Creates an additional offset that is subtracted from the "old" offset (Delta x-offset)
    }
}