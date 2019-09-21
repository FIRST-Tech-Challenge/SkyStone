package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
import static org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection.VUFORIA_KEY;

@Autonomous
public class TestTensorFlow extends AutoBase {
    protected static final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    protected VuforiaSkyStone vuforiaSkystone;
    protected TfodSkyStone tfodSkyStone;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public void runOpMode() {
        vuforiaSkystone = new VuforiaSkyStone();
        tfodSkyStone = new TfodSkyStone();

        initTfod();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfodSkyStone != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfodSkyStone.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel() == "Skystone") {
                                // telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                // recognition.getLeft(), recognition.getTop());
                                // telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                // recognition.getRight(), recognition.getBottom());
                                double angleToObject = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                telemetry.addData("Angle", angleToObject);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    protected void initTfod() {
        vuforiaSkystone.initialize(VUFORIA_KEY,
                VuforiaLocalizer.CameraDirection.BACK,
                false,
                true,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0,
                false);

        if(vuforiaSkystone != null) {
            tfodSkyStone.initialize(vuforiaSkystone, .7f, false, true);
            tfodSkyStone.activate();
        } else {
            telemetry.addData("status","not working");
            telemetry.update();
        }
    }
}