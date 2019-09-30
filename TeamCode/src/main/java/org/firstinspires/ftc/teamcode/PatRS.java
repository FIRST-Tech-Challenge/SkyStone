/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name="PatRS")
public class PatRS {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private final String VUFORIA_KEY = "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";

    public OpMode opmode;
    public HardwareMap hardwareMap;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public int goldCount = 0;
    public Recognition goldRecognition = null;
    public int goldCenter = 0;
    public int goldMiddle = 0;

    public void init(OpMode o) {
        opmode = o;
        hardwareMap = opmode.hardwareMap;

        // initialize Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // initialize TenserFlow Object Detector
      
        tparams.minimumConfidence = 0.20;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tparams, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SECOND_ELEMENT, LABEL_FIRST_ELEMENT);
    }

    public void activate() {
        if (tfod != null) tfod.activate();
    }

    public void stop() {
        if (tfod != null) tfod.shutdown();
    }

    public int scan() {
        goldCount = 0;
        // if we don't have an active object detector, return immediately
        if (tfod == null) return 0;

        // look for any new object recognition. If we don't see any, return immediately
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            // loop through whatever recognitions we have and record any gold we find
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                    // we found a gold mineral store it
                    goldCount++;
                    goldRecognition = recogniton;
                    goldCenter = (int)(recognition.getLeft() + recogniton.getRight()) / 2;
                    goldMiddle = (int)(recognition.getTop() + recogniton.getBottom()) / 2;
                }
            }
        }
        return goldCount;
    }

    public String toString() {
        String status = "gold=" + goldCount;
        if (goldCount > 0)
            status = status + " (" + goldCenter + "," + goldMiddle + ") "
                    + (int)goldRecognition.getWidth() + "x"
                    + (int)goldRecognition.getHeight();
        return status;
    }
}*/
