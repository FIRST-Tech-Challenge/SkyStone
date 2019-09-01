package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "AutoVuf", group = "Autonomous")
public class AutoVuf extends LinearOpMode {
    private KtRobot robot = new KtRobot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    List<Recognition> globalRecogn;

    private static final String VUFORIA_KEY = "AYCFV6H/////AAAAmekk6YIrl0eXiv6v/NSJkjV9eTSHNlS21VtbOvMpT/DU+eSVKRogBQ23cQ+qHItgdLdAyG0XIKpVWBewwHog581BgoBEyhLGzhnxI/57o+CYFi372QAb8faGRN/tE22Pm9BTinccijuCDITIS/9W4mQeUOGOMIC5rB76NZPeNT20Oj65AaG2s5N90hvh2+5xeQ4nhW3w34eez9C3tmO8A9ErqPG+CfDgKPhGZmI7SkAGvUlfzQDFvxPNeK8nYpD3ZnBYq+jytcTR5ch9MjrE0Oqbp5m+RnUIDNC7fP/4JPZ8l5i4JP6dvF1MAhpeJcAU2dIP7umddnO1M/mOOCZNwBD1o1qUMWjXSkvtBFtTstNl";

    int goldMineralX = -1;
    int silverMineral1X = -1;
    int silverMineral2X = -1;

    boolean detected = false;
    boolean dropped = false;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        robot.init(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null)
                tfod.activate();
            while (opModeIsActive()) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                if(!dropped) {
                    robot.drop();
                    dropped = true;
                }
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        telemetry.update();

                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1 && !detected) {
                            states();
                            detected = true;

                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //State Machine
    public void states() {
        String goldPos = "";
        Boolean center = false;
        Boolean analyze = true;
        int goldL = 1000, goldR = 0;
        while (analyze) {
            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)//far left
                goldPos = "left";
            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)//far right
                goldPos = "right";
            else
                goldPos = "center";
            sleep(250);
            switch (goldPos) {
                case "left":
                    tfod.shutdown();
                    telemetry.addData("Gold Mineral Position", "Left");
                    telemetry.update();
                    //robot.swing(robot.getSwingUp());
                    robot.brake();
                    sleep(1000);
                    robot.brake();
                    robot.rightMove(0.5); //<-
                    robot.leftMove(-0.11);
                    sleep(850);
                    robot.brake();
                    drive(0.5, 2700);//encloses cube
                    robot.brake();
                    sleep(500);
                    //telemetry.addData("FLAG", "DOWN");
                    //telemetry.update();
                    sleep(500);
                    robot.brake();
                    sleep(400);
                    robot.leftMove(0.5);
                    robot.rightMove(-0.1);
                    sleep(1950);
                    robot.brake();
                    sleep(1000);
                    drive(0.25, 4500);//drives to depot
                    robot.swing(robot.getSwingUp()); //down
                    drive(-1.0, 550);//drops token
                    robot.swing(robot.getSwingDown()); //up
                    drive(0.5, 1000);
                    robot.brake();
                    //robot.getCube(goldPos);
                    analyze = false;
                    break;
                case "center":
                    tfod.shutdown();
                    telemetry.addData("Gold Mineral Position", "Center");
                    telemetry.update();
                    sleep(1000);
                    robot.getCube(goldPos);
                    analyze = false;
                    break;
                case "right":
                    tfod.shutdown();
                    telemetry.addData("Gold Mineral Position", "Right");
                    telemetry.update();
                    //robot.swing(robot.getSwingUp());
                    robot.brake();
                    sleep(1000);
                    robot.brake();
                    robot.leftMove(0.5); //<-
                    robot.rightMove(-0.11);
                    sleep(1000);
                    robot.brake();
                    drive(0.5, 2700);//encloses cube
                    robot.brake();
                    sleep(500);
                    telemetry.addData("FLAG", "DOWN");
                    telemetry.update();
                    sleep(500);
                    robot.brake();
                    sleep(400);
                    robot.rightMove(0.5);
                    robot.leftMove(-0.1);
                    sleep(1950);
                    robot.brake();
                    sleep(1000);
                    drive(0.25, 4500);//drives to depot
                    robot.swing(robot.getSwingUp()); //down
                    drive(-0.9, 550);//drops token
                    robot.swing(robot.getSwingDown()); //up
                    drive(0.5, 1000);
                    robot.brake();
                    stop();
                    //robot.getCube(goldPos);
                    analyze = false;
                    break;
                default:
                    analyze = true;//restarts the check

            }
        }

        telemetry.update();
        tfod.deactivate();
    }


    /**
     * Initialize the Vuforia localization engine.
     */

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void drive(double pow, long sleepT) {
        robot.drive(pow);
        sleep(sleepT);
        robot.brake();
    }
}