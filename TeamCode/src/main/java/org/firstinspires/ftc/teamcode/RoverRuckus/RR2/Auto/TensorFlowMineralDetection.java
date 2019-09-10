package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
@Deprecated
public class TensorFlowMineralDetection {
    public enum Location{
        CENTER,LEFT,RIGHT,UNKNOWN;
    }

    public enum MineralViewType{
        ALL3, LEFT2, RIGHT2;
    }

    public HardwareMap hardwareMap;
    public TensorFlowMineralDetection(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public Location location;
    public MineralViewType mineralViewType;
    public Telemetry telemetry;
    public LinearOpMode linearOpMode;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VUFORIA_KEY = "AQOQmUj/////AAABmXCrndWask2hud2XzSvIrY52Fjls7yhcRKcQMVgooTqhv7MCYikM1d8E8z5LHk0DHRlwk+Qwvhl1k+p6NIRSQ4dHtbxYxpD0nO9pEhtlY/ABsjRyS+QrC3xqImfkY+IL6zNXtySZjozAhoNmP2sIx7JBN6hcpdabhrywRplCBfOh2uUI3FLMD544Lo6BIHST42mTExPyUIRmCLf4JHEavNAa3cC19X8IRzfm7cWlKLbAJCzNls2Tkp/wkUAdRSBdDQ4156qBsiIC5XoFVhdz+M7o62+MqKlDa6bm+VcYvex8gkAwRaoiOYGyzIdVLvbnbAAdAdjApeDozBzukAjsjF3tPltcVYyIJRM5mRWeNJGj";

    public VuforiaLocalizer vuforia;

    public TFObjectDetector tfod;

    public WebcamName webcamName;

    int goldXPos = -1;
    double scale = 0;
    int skipFirstMineral = 0;

    public Location runObjectDetection() throws VuforiaException{
        if (tfod != null) {
            tfod.activate();
        }
        long startTime = SystemClock.elapsedRealtime();

        while (this.location != Location.UNKNOWN && linearOpMode.opModeIsActive() && (SystemClock.elapsedRealtime() - startTime) < 2000) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                    //(0,0) is topLeft corner
                    Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                        @Override
                        public int compare(Recognition lhs, Recognition rhs) {
                            if(lhs.getBottom()<rhs.getBottom()){
                                return 1;
                            }else if(lhs.getBottom()>rhs.getBottom()){
                                return -1;
                            }else {
                                return 0;
                            }
                        }
                    });

                    ArrayList<Recognition> updated = new ArrayList<>(updatedRecognitions);
                    List<Recognition> updatedList = updated.subList(0,Math.min(3,updated.size()));
                    telemetry.addLine(updatedList.toString());
                    telemetry.update();

                    Collections.sort(updatedList, new Comparator<Recognition>() {
                        @Override
                        public int compare(Recognition lhs, Recognition rhs) {
                            if(lhs.getLeft()<rhs.getLeft()){
                                return -1;
                            }else if(lhs.getLeft()>rhs.getLeft()){
                                return 1;
                            }else {
                                return 0;
                            }
                        }
                    });

                    telemetry.addLine("LIST RAW: "+ updatedList.toString());

                    for(int i = 0;i<updatedList.size()-1;i++){
                        if(updatedList.get(i).getBottom()/updatedList.get(i+1).getBottom() > 1.25){
                            updatedList.remove(i+1);
                        }else if(updatedList.get(i+1).getBottom()/updatedList.get(i).getBottom() > 1.25){
                            updatedList.remove(i);
                        }
                    }

                    if(updatedList.size() == 2){
                        scale = updatedList.get(0).getLeft()/(800-updatedList.get(1).getLeft());

                        if(scale<0.9 && scale>0.2){
                            this.location = Location.CENTER;
                            return Location.CENTER;
                        }
                        else if(scale < 0.4){
                            this.mineralViewType = MineralViewType.LEFT2;
                            goldXPos = 2;

                        }else if(scale>1.1){
                            this.mineralViewType = MineralViewType.RIGHT2;
                            goldXPos = 0;
                            skipFirstMineral= 1;
                        }
                    }

                    for(int i = 0;i<updatedList.size();i++){
                        if(updatedList.get(i).getLabel().equals(LABEL_GOLD_MINERAL)){
                            goldXPos = i+skipFirstMineral;
                        }
                    }

                    telemetry.addLine(updatedList.toString());
                    telemetry.update();

                    switch (goldXPos){
                        case 0:
                            this.location = Location.LEFT;
                            break;
                        case 1:
                            this.location = Location.CENTER;
                            break;
                        case 2:
                            this.location = Location.RIGHT;
                            break;
                        default:
                            this.location = Location.UNKNOWN;
                            break;
                    }

                    telemetry.addLine(updatedList.toString());
                    telemetry.addLine("Scale: " + Double.toString(scale));

                    if(updatedList.size() == 1){
                        Recognition recognition = updatedList.get(0);
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                            if(recognition.getLeft()<300){
                                this.location = Location.LEFT;
                            }else if(recognition.getLeft()>500){
                                this.location = Location.RIGHT;
                            }else{
                                this.location = Location.CENTER;
                            }
                        }else{
                            if(this.location == Location.RIGHT){
                                this.location = Location.LEFT;
                            }else if(this.location == Location.LEFT){
                                this.location = Location.RIGHT;
                            }else{
                               this.location = Location.LEFT;
                            }
                        }
                    }

                    tfod.deactivate();
                    tfod.shutdown();
                    return location;
                }
            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }
        location = Location.UNKNOWN;
        return location;
    }

    public void initVuforia() throws VuforiaException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.cameraDirection = CameraDirection.BACK;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}