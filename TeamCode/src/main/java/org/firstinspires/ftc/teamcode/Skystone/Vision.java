package org.firstinspires.ftc.teamcode.Skystone;

import android.drm.DrmStore;
import android.graphics.Bitmap;
import android.net.wifi.WifiManager;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Vision{
    private final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    float value = 0;

    public enum Location{
        CENTER,LEFT,RIGHT,UNKNOWN;
    }

    private Location location = Location.UNKNOWN;
    private Robot robot;

    private final double minConfidenceLevel = 0.2;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public Vision(Robot robot){
        this.robot = robot;
        initVision();
    }

    public void initVision(){
        initVuforia();
        initTfod();
        tfod.activate();
    }

    public Location runDetection(){
        long startTime = SystemClock.elapsedRealtime();

        // 2 is right, 1 is center, 0 is left
        ArrayList<Double> leftCount = new ArrayList<>();
        ArrayList<Double> centerCount = new ArrayList<>();
        ArrayList<Double> rightCount = new ArrayList<>();

        while (robot.getLinearOpMode().opModeIsActive() && SystemClock.elapsedRealtime()-startTime<500){

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()>0) {
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        return (int)(recognition.getConfidence()-t1.getConfidence());
                    }
                });

                // For every detection, see if its confidence level is greater than .5. If so, find the width of the detection and store the shortest detection.
                for (int i = 0; i < updatedRecognitions.size(); i++){
                    float blockPixelX = (updatedRecognitions.get(i).getTop()+updatedRecognitions.get(i).getBottom())/2;
                    float blockPixelY = (updatedRecognitions.get(i).getLeft() + updatedRecognitions.get(i).getRight())/2;

                    robot.getTelemetry().addLine("blockPixelX: " + blockPixelX);
                    robot.getTelemetry().addLine("blockPixelY: " + blockPixelY);
                    robot.getTelemetry().addLine("blockLabel: " + updatedRecognitions.get(i).getLabel());

                    if (350 < blockPixelY && blockPixelY < 450){
                        if ((double)updatedRecognitions.get(i).getConfidence() > 0.95 && updatedRecognitions.get(i).getLabel().equals("Skystone")){
                            if (blockPixelX < 600){
                                return Location.RIGHT;
                            } else if (blockPixelX < 800){
                                return Location.CENTER;
                            } else {
                                return Location.LEFT;
                            }
                        }
                        // otherwise if it is greater than 50% confidence add its confidence to the sum
                        else if ((double)updatedRecognitions.get(i).getConfidence() > minConfidenceLevel && updatedRecognitions.get(i).getLabel().equals("Skystone")) {
                            if (blockPixelX < 600){
                                rightCount.add((double)updatedRecognitions.get(i).getConfidence());
                            } else if (blockPixelX < 800) {
                                centerCount.add((double)updatedRecognitions.get(i).getConfidence());
                            } else {
                                leftCount.add((double)updatedRecognitions.get(i).getConfidence());
                            }
                        }
                    }
                }

            }

        }

        // find the confidence levels
        double leftConfidence = MathFunctions.arrayListAverage(leftCount);
        double centerConfidence = MathFunctions.arrayListAverage(centerCount);
        double rightConfidence = MathFunctions.arrayListAverage(rightCount);

        robot.getTelemetry().addLine("left confidence: " + leftConfidence + " leftCount: " + leftCount.size());
        robot.getTelemetry().addLine("right confidence: " + rightConfidence + " rightCount: " + rightCount.size());
        robot.getTelemetry().addLine("center confidence: " + centerConfidence + " centerCount: " + centerCount.size());

        if (leftConfidence == rightConfidence){
            return Location.CENTER;
        }

        // all of this just determines the best just based on the confidence levels
        if (leftConfidence > centerConfidence) {
            // if left confidence is more than center and right
            if (leftConfidence > rightConfidence) {
                return Location.LEFT;
            }
            // if left confidence is more than center but right is more than left
            else {
                return Location.RIGHT;
            }
        } else {
            // if center confidence is more than left and more than right
            if (centerConfidence > rightConfidence){
                return Location.CENTER;
            }
            // if center confidence is more left but right is more than center
            else {
                return Location.RIGHT;
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //  Instantiate the Vuforia engine

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        final String TFOD_MODEL_ASSET = "Skystone.tflite";
        final String LABEL_FIRST_ELEMENT = "Stone";
        final String LABEL_SECOND_ELEMENT = "Skystone";
        int tfodMonitorViewId = robot.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = minConfidenceLevel;

        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
