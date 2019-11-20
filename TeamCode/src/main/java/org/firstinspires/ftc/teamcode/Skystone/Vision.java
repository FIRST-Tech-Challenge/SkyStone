package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Vision extends LinearOpMode{
    private enum Location{
        CENTER,LEFT,RIGHT,UNKNOWN;
    }

    Location location = Location.UNKNOWN;
    Robot robot;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode(){}

    public Vision(Robot robot){
        this.robot = robot;
    }

    public void initTensorflow(){
        initVuforia();
        initTfod();

        tfod.activate();
    }

    final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    public Location detectTensorflow(){
        double shortestDetectionLength = 100000;
        float shortestValue = 700;
        double difference = 0;

        /**
         * if it sees something/object detected : get the confidence
         * if confidence is greater than 0.7 : find its position and return that
         * if confidence is less than 0.7 : get the position it thinks, go through the code again, if is the same, then return that
         */

        long startTime = SystemClock.elapsedRealtime();
        // scan for 5 seconds
        while (robot.getLinearOpMode().opModeIsActive() && SystemClock.elapsedRealtime()-startTime < 6000){
            // get all the detections
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            // if there is a detection run the logic
            if (updatedRecognitions != null && updatedRecognitions.size()>0) {
                // sorts based on confidence levels
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        return (int)(recognition.getConfidence()-t1.getConfidence());
                    }
                });

                for (int i = 0; i < updatedRecognitions.size(); i++){
                    if (updatedRecognitions.get(i).getConfidence() > 0.5){
                        if (updatedRecognitions.get(i).getTop()-updatedRecognitions.get(i).getBottom() < shortestDetectionLength) {
                            difference = shortestDetectionLength - (updatedRecognitions.get(i).getTop()-updatedRecognitions.get(i).getBottom());
                            shortestDetectionLength = updatedRecognitions.get(i).getTop()-updatedRecognitions.get(i).getBottom();
                            shortestValue = (updatedRecognitions.get(i).getTop()+updatedRecognitions.get(i).getBottom())/2;
                        }
                    }
                }
                if (difference < 100){
                    location = Location.CENTER;
                } else {
                    if (shortestValue > 900) {
                        location = Location.LEFT;
                    } else if (shortestValue > 600){
                        location = Location.CENTER;
                    } else{
                        location = Location.RIGHT;
                    }
                }
                return location;


                // iterate through each recognition
//                for (int i = 0; i < updatedRecognitions.size(); i++){
//                    // value is the center of the detection
//                    float value = (updatedRecognitions.get(i).getTop()+updatedRecognitions.get(i).getBottom())/2;
//                    // if the confidence is greater than 0.9, then return that
//                    if ((double)updatedRecognitions.get(i).getConfidence() > 0.9){
//                        if (value < 600){
//                            return new Detection(2, value, updatedRecognitions.size());
//                        } else if (value < 800){
//                            return new Detection(1, value, updatedRecognitions.size());
//                        } else {
//                            return new Detection(0, value, updatedRecognitions.size());
//                        }
//                    }
//                    // if the confidence is greater than 0.5, add it to the arraylist
//                    if ((double)updatedRecognitions.get(i).getConfidence() > 0.5) {
//                        values.add(value);
//                        if (value < 560){
//                            retVals.add(2);
//                        } else if (value < 605) {
//                            retVals.add(1);
//                        } else {
//                            retVals.add(0);
//                        }
//                    }
//                }
            }
        }
//        // find the average of everything in the arraylist
//        double retVal = 0;
//        for (int i = 0; i < retVals.size(); i++){
//            retVal += retVals.get(i);
//        }
//        retVal /= retVals.size();
//        float valueAvg = 0;
//        for (int i = 0; i < values.size(); i++){
//            valueAvg += values.get(i);
//        }
//        valueAvg /= values.size();
////        telemetry.addLine("retVal" + retVal);
////        telemetry.update();
//        // return rounded int average
//        return new Detooection((int)Math.round(retVal), valueAvg, 0);
        return location;
    }

    public void initVuforia() {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        final String TFOD_MODEL_ASSET = "Skystone.tflite";
        final String LABEL_FIRST_ELEMENT = "Stone";
        final String LABEL_SECOND_ELEMENT = "Skystone";
        int tfodMonitorViewId = robot.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.50;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
