package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.ethan_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Deprecated
@Disabled
@Autonomous
public class tensorFlow extends LinearOpMode {
    protected String goldPosition;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static String VUFORIA_KEY = "AQOQmUj/////AAABmXCrndWask2hud2XzSvIrY52Fjls7yhcRKcQMVgooTqhv7MCYikM1d8E8z5LHk0DHRlwk+Qwvhl1k+p6NIRSQ4dHtbxYxpD0nO9pEhtlY/ABsjRyS+QrC3xqImfkY+IL6zNXtySZjozAhoNmP2sIx7JBN6hcpdabhrywRplCBfOh2uUI3FLMD544Lo6BIHST42mTExPyUIRmCLf4JHEavNAa3cC19X8IRzfm7cWlKLbAJCzNls2Tkp/wkUAdRSBdDQ4156qBsiIC5XoFVhdz+M7o62+MqKlDa6bm+VcYvex8gkAwRaoiOYGyzIdVLvbnbAAdAdjApeDozBzukAjsjF3tPltcVYyIJRM5mRWeNJGj";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        waitForStart();

        detectGold();
    }

    protected void detectGold() {
        if (tfod != null) {
            tfod.activate();
        }

        while (opModeIsActive()) {
            if (tfod != null) {
                // Retrieve new information from tfod
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) { // Test to see if there is any new information
                    telemetry.addData("Minerals Detected", updatedRecognitions.size());

                    int GoldXPos = -1;
                    int Silver1_XPos = -1;
                    int Silver2_XPos = -1;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            GoldXPos = (int) recognition.getLeft();
                        } else if (Silver1_XPos == -1) {
                            Silver1_XPos = (int) recognition.getLeft();
                        } else if (Silver2_XPos == -1) {
                            Silver2_XPos = (int) recognition.getLeft();
                        }
                    }
                    if (GoldXPos != -1 && Silver1_XPos != -1 && Silver2_XPos != 1) {
                        if (GoldXPos < Silver1_XPos && GoldXPos < Silver2_XPos) {
                            goldPosition = "Left";
                        } else if (GoldXPos > Silver1_XPos && GoldXPos > Silver2_XPos) {
                            goldPosition = "Right";
                        } else {
                            goldPosition = "Center";
                        }
                    }
                }

                telemetry.addData("Gold position", goldPosition);
                telemetry.update();
            }
        }
    }

    protected void initVuforia() {
        // Create a vuforia instance for us to pass parameters into
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Pass in the parameters required for vuforia
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(CameraName.class, "Webcam 1");

        // Set up vuforia, passing in the parameters we have set up
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
