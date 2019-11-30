package org.firstinspires.ftc.teamcode.Skystone;

import android.drm.DrmStore;
import android.graphics.Bitmap;
import android.net.wifi.WifiManager;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class Vision {
    private final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    float value = 0;

    public enum Location{
        CENTER,LEFT,RIGHT,UNKNOWN;
    }

    private Location location = Location.UNKNOWN;

    private final double minConfidenceLevel = 0.3;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    private LinearOpMode linearOpMode;

    public Vision(LinearOpMode linearOpmode){
        this.linearOpMode = linearOpmode;
        initVision();
    }

    public void initVision(){
        initVuforia();
        initTfod();
        tfod.activate();
    }

    public Location runDetection(){
        return runDetection(true);
    }

    public Location runDetection(boolean deactivate){
        long startTime = SystemClock.elapsedRealtime();

        // 2 is right, 1 is center, 0 is left
        ArrayList<Double> leftCount = new ArrayList<>();
        ArrayList<Double> centerCount = new ArrayList<>();
        ArrayList<Double> rightCount = new ArrayList<>();

        while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime()-startTime<2000){

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
//
//                    linearOpMode.telemetry.addLine("blockPixelX: " + blockPixelX);
//                    linearOpMode.telemetry.addLine("blockPixelY: " + blockPixelY);
//                    linearOpMode.telemetry.addLine("blockLabel: " + updatedRecognitions.get(i).getLabel());

                    if (350 < blockPixelY && blockPixelY < 450){
                        if ((double)updatedRecognitions.get(i).getConfidence() > 0.95 && updatedRecognitions.get(i).getLabel().equals("Skystone")){
                            tfod.deactivate();
                            tfod.shutdown();
                            if (blockPixelX < 690){
                                return Location.RIGHT;
                            } else if (blockPixelX < 810){
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
        if (deactivate){
            tfod.deactivate();
            tfod.shutdown();
        }
        // find the confidence levels
        double leftConfidence = MathFunctions.arrayListAverage(leftCount);
        double centerConfidence = MathFunctions.arrayListAverage(centerCount);
        double rightConfidence = MathFunctions.arrayListAverage(rightCount);

//        linearOpMode.telemetry.addLine("left confidence: " + leftConfidence + " leftCount: " + leftCount.size());
//        linearOpMode.telemetry.addLine("right confidence: " + rightConfidence + " rightCount: " + rightCount.size());
//        linearOpMode.telemetry.addLine("center confidence: " + centerConfidence + " centerCount: " + centerCount.size());

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

    public Location runDetection2(boolean deactivated) {

        ArrayList<DetectionResult> detectionResults = new ArrayList<>();
        long startTime = SystemClock.elapsedRealtime();

        int numOfSkystonesFound = 0;

        DetectionResult center = new DetectionResult(Location.CENTER);
        DetectionResult right = new DetectionResult(Location.RIGHT);
        DetectionResult left = new DetectionResult(Location.LEFT);

        detectionResults.add(center);
        detectionResults.add(right);
        detectionResults.add(left);

        while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime() - startTime < 2000 && numOfSkystonesFound <= 3) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        //sort in descending order
                        if (recognition.getConfidence() < t1.getConfidence()){
                            return 1;
                        } else if (recognition.getConfidence() == t1.getConfidence()) {
                            return 0;
                        } else {
                            return -1;
                        }
                    }
                });

                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    Recognition recognition = updatedRecognitions.get(i);
                    if (recognition.getLabel().equals("Skystone")){
                        numOfSkystonesFound++;
                        float blockPixelX = (updatedRecognitions.get(i).getTop()+updatedRecognitions.get(i).getBottom())/2;
                        float blockPixelY = (updatedRecognitions.get(i).getLeft() + updatedRecognitions.get(i).getRight())/2;

//                        linearOpMode.telemetry.update();
                        if (blockPixelY < 100){
                            if (blockPixelX > 100) {
                                linearOpMode.telemetry.clear();
                                linearOpMode.telemetry.addLine("blockPixelX: " + blockPixelX + " blockPixelY: " + blockPixelY + " confidence:" + recognition.getConfidence());
                                linearOpMode.telemetry.addLine("updated");
                                if (blockPixelX > 900) {
                                    left.incrementConfidence(recognition.getConfidence());
                                } else if (blockPixelX > 700) {
                                    center.incrementConfidence(recognition.getConfidence());
                                } else {
                                    right.incrementConfidence(recognition.getConfidence());
                                }
                            }
                        }
                    }
                }



            }
        }

        // Sort detectionResult values (left, right, center) by most count. If two of those have
        // the same count, then sort them by average confidence.
        Collections.sort(detectionResults, new Comparator<DetectionResult>() {
            @Override
            public int compare(DetectionResult detectionResult, DetectionResult t1) {
                if (detectionResult.avgConfidence() < t1.avgConfidence()){
                    return 1;
                } else if (detectionResult.avgConfidence() > t1.avgConfidence()){
                    return -1;
                } else {
                    if (detectionResult.getCount() < t1.getCount()){
                        return 1;
                    }else if (detectionResult.getCount() > t1.getCount()){
                        return -1;
                    }else {
                        return 0;
                    }
                }
            }
        });

        if (numOfSkystonesFound == 0){
            return Location.UNKNOWN;
        }else {
            return detectionResults.get(0).getLocation();
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
        int tfodMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = minConfidenceLevel;

        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}

