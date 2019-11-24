package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.BLOCKPOS;
import org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.SkystonePosition;

import static org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.BLOCKPOS.NONE;

@Autonomous(name = "Test Auto (DUMMY)", group = "Linear Opmode")
public class testAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private SkystonePosition posFinder = new SkystonePosition();
    public static BLOCKPOS position = BLOCKPOS.NONE;


    double power = 0.5;

    @Override
    public void runOpMode() {
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        if (tfod != null) {
            tfod.activate();
        }

        position = detectPosition();

        String status = "";
        if (position == BLOCKPOS.LEFT) {
            status = "Left";
        } else if (position == BLOCKPOS.MIDDLE) {
            status = "Middle";
        } else if (position == BLOCKPOS.RIGHT) {
            status = "Right";
        } else {
            status = "Null";
        }

        telemetry.addData("status:", status);
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();
        while (opModeIsActive()) {
            if (position == BLOCKPOS.LEFT) {
                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(-power);

                sleep(500);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(-power);
                hwMap.backRight.setPower(power);

                sleep(500);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(power);
                hwMap.rightIntake.setPower(-power);

                sleep(1000);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);
            }
            else if(position == BLOCKPOS.MIDDLE) {
                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(-power);
                hwMap.backRight.setPower(power);

                sleep(500);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(power);
                hwMap.rightIntake.setPower(-power);

                sleep(1000);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);
            }
            else{
                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(-power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(power);

                sleep(500);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(-power);
                hwMap.backRight.setPower(power);

                sleep(500);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(power);
                hwMap.rightIntake.setPower(-power);

                sleep(1500);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "WebcamFront");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public BLOCKPOS detectPosition() {
        BLOCKPOS skystonePos = BLOCKPOS.NONE;
        while (skystonePos == NONE) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }

                    skystonePos = posFinder.findLocation(updatedRecognitions);
                }
            }
        }

        if(tfod == null){
            tfod.shutdown();
        }

        return skystonePos;
    }
}