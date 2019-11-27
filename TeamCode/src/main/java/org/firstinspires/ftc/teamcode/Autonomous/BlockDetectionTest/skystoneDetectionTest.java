package org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "TensorFlow Skystone Detection Test", group = "Linear Opmode" )
@Disabled
public class skystoneDetectionTest extends LinearOpMode{
    public static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    public static final String LABEL_FIRST_ELEMENT = "Skystone";
    public static final String LABEL_SECOND_ELEMENT = "Stone";

    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private SkystonePosition posFinder = new SkystonePosition();

    @Override
    public void runOpMode(){
        initVuforia();
        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            initTfod();
        }else{
            telemetry.addData("Sorry!", "this device is not compatible with TFOD");
        }
        if(tfod != null){
            tfod.activate();
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()){
            while(opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        BLOCKPOS position = posFinder.findLocation(updatedRecognitions);
                        String status = "";
                        if (position == BLOCKPOS.LEFT){
                            status = "Left";
                        }else if(position == BLOCKPOS.MIDDLE){
                            status = "Middle";
                        }else if(position == BLOCKPOS.RIGHT){
                            status = "Right";
                        }else{
                            status = "Null";
                        }
                        telemetry.addData("status:", status);
                        telemetry.update();
                    }

                }
            }
        }

        if(tfod == null){
            tfod.shutdown();
        }
    }
    private void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }
    private void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

}
