
// Modified by Aidan Sun on 24 October 2019
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "WebcamTest")
public class WebcamTest extends LinearOpMode {
    /* Constants */
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final double MIN_CONFIDENCE = 0.8;
    private static final int NUM_BLOCKS = 3;

    /* Arrays for skystone position detection */
    private static float[] left_vals = {0, 0, 0};
    private static String[] label_vals = {"", "", ""};

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWkYdzb/////AAABmWsW4urs2EjOmK3aBEWJEFxEbGHh6an3+HFdzPIUoFs9oQJRm34hA528YQTZohbGdiZH5Gyot22SPfkQyhOgQMXlgfiQ0aTeNMSlD/FwWJ9kAjP1vIfFYnhEs9TjmpmPxKQD9OsxeA7hoKWgQWrOlNu0yG21oEWsr3FEh0rbAazVcNW27TFw5X1rK9mRdZT1/acPcntsbM74Sr/iy/nrp9y/rufNfwpzhnzkgFSJ6k5+6u6VEpE5Rg6M8kM6kfp+SCVNtqeYuiMV3RvKHoKuG4k+IOxd4uteb33+w8VNbIzoCMhuEP0iyb5hX+t5b6Tg6syLzkD5q9znjePjCxWx0MTFNtNe87Yvh2NBCYEwuOEo";

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();
        }

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == NUM_BLOCKS) { // Camera detected a certain number of blocks
                            //telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                /*
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                        */
                                label_vals[i] = recognition.getLabel();
                                left_vals[i] = recognition.getLeft();
                                i++; // Bug Fix - Counter Increment
                            }
                            String position;
                            position = getSkystonePosition(label_vals, left_vals);
                            switch (position) {
                                case "left":
                                    goLeft();
                                    break;
                                case "center":
                                    goCenter();
                                    break;
                                case "right":
                                    goRight();
                                    break;
                            }
                            //telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void InsertionSort(float[] arr)
    {
        int n = arr.length;
        for (int i = 1; i < n; ++i) {
            float key = arr[i];
            int j = i - 1;

            /* Move elements of arr[0..i-1], that are
               greater than key, to one position ahead
               of their current position */
            while (j >= 0 && arr[j] > key) {
                arr[j + 1] = arr[j];
                j = j - 1;
            }
            arr[j + 1] = key;
        }
    }

    private String getSkystonePosition(String[] labels, float[] vals) {
        float skystoneVal = 0;
        String pos = "";

        for (int i = 0; i < 3; i++) {
            String object = labels[i];
            if (object.equals(LABEL_SECOND_ELEMENT)) {
                skystoneVal = vals[i]; // Get the left position of the skystone
                break; // Exit the loop
            }
        }

        InsertionSort(vals); // Sort the array - smallest values in front
        float val1 = vals[0];
        float val2 = vals[1];
        float val3 = vals[2];
        /* Find the placement of the skystone once values have been sorted */
        if (skystoneVal == val1) { // Left side
            pos = "left";
        } else if (skystoneVal == val2) { // Center
            pos = "center";
        } else if (skystoneVal == val3) { // Right
            pos = "right";
        }
        return pos;
    }

    /* Functions for robot driving stuff*/
    private void goLeft() {
        // Stuff to do if skystone is on left side
    }

    private void goCenter() {
        // Stuff to do if skystone is in the center
    }

    private void goRight() {
        // Stuff to do if skystone is on right side
    }

    /*
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /*
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}