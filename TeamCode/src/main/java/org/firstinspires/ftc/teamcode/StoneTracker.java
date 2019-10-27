package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


public class StoneTracker {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY ="AQlSsBn/////AAABmU/p6enRBUR1jH8Y1lsJSdYnIQtgf8kMy4EMI11VGbQgZhO8Mh5bRwYHGRyjTD9EoyHO6TJuoWhKWYc2FtPlIElB73QYHU29ZvFoJxwJeXyNgBkZO5frHNtHxOPkrSfCpZonmgx7wYmxJ32Yt6Uw6PFA0ksFTPDYPPweNHa67fgt5t3OvNtwTzabXEW11r3BrwwzwJUbCPqYRzleG9ys93SPqIBLDs03XbfLjvflXWUFKKVwcSwYOuJJUUOOveD9FZqaTu9jnx7o9VCw3LeJVVVrAT8ParHvE5HUhXwDaSeSVZa0Y6LsFI0mkAiiiP6uD2Kw/iDPDdtBxOaNyRjv7x/ILjO2y/VkaDsEd53rJTx7";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    public List<Recognition> updatedRecognitions = null;
    public int stonecount = 0;
    public Recognition stone = null;
    public Recognition skystone = null;
    
    public void init(HardwareMap hardwareMap) {
        // initialize the Vuforia engine
        VuforiaLocalizer.Parameters vparams = new VuforiaLocalizer.Parameters();
        vparams.vuforiaLicenseKey = VUFORIA_KEY;
        vparams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vparams);
        
        // initialize TensorFlow Object Detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void activate() {
        if (tfod != null) tfod.activate();
    }

    public void stop() {
        if (tfod != null) tfod.shutdown();
    }    

    public int scan() {
        if (tfod == null) return 0;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            stone = null; 
            skystone = null;
            for (Recognition recognition : updatedRecognitions) {
                stone = recognition;
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                   skystone = recognition;
                }
            }
            stonecount = updatedRecognitions.size();
            return stonecount;
        }
        return 0;
    }
    
    public String toString() {
        if (skystone != null) {
            return String.format("%s @ (%.0f,%.0f) %.0fx%.0f %.2fdeg",
                skystone.getLabel(),
                skystone.getLeft() + stone.getWidth()/2,
                skystone.getTop() + stone.getHeight()/2,
                skystone.getWidth(),
                skystone.getHeight(),
                skystone.estimateAngleToObject(AngleUnit.DEGREES));
        }
        if (stone != null) {
            return String.format("%s @ (%.0f,%.0f) %.0fx%.0f %.2fdeg",
                stone.getLabel(),
                stone.getLeft() + stone.getWidth()/2,
                stone.getTop() + stone.getHeight()/2,
                stone.getWidth(),
                stone.getHeight(),
                stone.estimateAngleToObject(AngleUnit.DEGREES));
        }
        return "(none)";
    }
   
}
