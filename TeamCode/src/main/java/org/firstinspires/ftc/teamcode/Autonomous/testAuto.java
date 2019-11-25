package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.BLOCKPOS;
import org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.SkystonePosition;

import static org.firstinspires.ftc.teamcode.Autonomous.BlockDetectionTest.BLOCKPOS.NONE;

@Autonomous(name = "Test Auto (DUMMY)", group = "Linear Opmode")
@Disabled
public class testAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    private static final String LABEL_FIRST_ELEMENT = "Skystone";
    private static final String LABEL_SECOND_ELEMENT = "Stone";

    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private SkystonePosition posFinder = new SkystonePosition();
    public double centerPos = 0;
    public BLOCKPOS position = BLOCKPOS.LEFT;
    public HardwareMap hwMap;
    public double imgWidth;

    double power = 0.4;

    @Override
    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE); //???
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        if (tfod != null) {
            tfod.activate();
        }

        /*position = detectPosition();*/

        /*String status;
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
        telemetry.update();*/


        /** Wait for the game to begin */
        waitForStart();
        if (opModeIsActive()) {
            boolean foundSkystone = false;
            if (position == BLOCKPOS.LEFT) {
                /*hwMap.frontLeft.setPower(-0.);
                hwMap.frontRight.setPower(0.4);
                hwMap.backLeft.setPower(0.4);
                hwMap.backRight.setPower(-0.4);
                sleep(800);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);


                hwMap.frontLeft.setPower(power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(power);

                sleep(900);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                intake(1,3600);

                hwMap.frontLeft.setPower(0.2);
                hwMap.frontRight.setPower(0.2);
                hwMap.backLeft.setPower(0.2);
                hwMap.backRight.setPower(0.2);

                sleep(750);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);/*

                 */
                hwMap.frontLeft.setPower(power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(power);

                sleep(900);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                intake(1, 10000);

                hwMap.frontLeft.setPower(0.2);
                hwMap.frontRight.setPower(0.2);
                hwMap.backLeft.setPower(0.2);
                hwMap.backRight.setPower(0.2);

                sleep(900);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);

                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(-power);
                hwMap.backLeft.setPower(-power);
                hwMap.backRight.setPower(-power);

                sleep(750);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                sleep(800);

                hwMap.frontLeft.setPower(0.5);
                hwMap.frontRight.setPower(-0.5);
                hwMap.backLeft.setPower(-0.5);
                hwMap.backRight.setPower(0.5);

                while (!foundSkystone) {

                    centerPos = detectPosition();

                    if ((imgWidth / 2 - 100 <= centerPos) && (centerPos <= imgWidth / 2 + 100)) {
                        foundSkystone = true;
                        telemetry.addData("Detected", centerPos + " " + imgWidth / 2);
                        telemetry.update();
                        hwMap.frontLeft.setPower(0);
                        hwMap.frontRight.setPower(0);
                        hwMap.backLeft.setPower(0);
                        hwMap.backRight.setPower(0);

                        intake(1, 10000);

                        hwMap.frontLeft.setPower(0.2);
                        hwMap.frontRight.setPower(0.2);
                        hwMap.backLeft.setPower(0.2);
                        hwMap.backRight.setPower(0.2);

                        sleep(2500);

                        hwMap.frontLeft.setPower(0);
                        hwMap.frontRight.setPower(0);
                        hwMap.backLeft.setPower(0);
                        hwMap.backRight.setPower(0);
                    }


                }


            } else if (position == BLOCKPOS.MIDDLE) {
                hwMap.leftIntake.setPower(-1);
                hwMap.rightIntake.setPower(1);

                hwMap.frontLeft.setPower(power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(power);

                sleep(1500);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);
                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);
            } else {
                hwMap.frontLeft.setPower(-power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(-power);

                sleep(450);

                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);

                hwMap.leftIntake.setPower(-1);
                hwMap.rightIntake.setPower(1);

                hwMap.frontLeft.setPower(power);
                hwMap.frontRight.setPower(power);
                hwMap.backLeft.setPower(power);
                hwMap.backRight.setPower(power);

                sleep(1500);

                hwMap.leftIntake.setPower(0);
                hwMap.rightIntake.setPower(0);
                hwMap.frontLeft.setPower(0);
                hwMap.frontRight.setPower(0);
                hwMap.backLeft.setPower(0);
                hwMap.backRight.setPower(0);
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

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        VuforiaLocalizer.CloseableFrame frame = null;

        try {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch (Exception e) {
            e.printStackTrace();
        }
        imgWidth = frame.getImage(0).getWidth();

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

    public List<Recognition> recognize() {
        List<Recognition> updatedRecognitions = null;
        if (tfod != null) {
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
                // step through the list of recognitions and display boundary info.
            }
        }
        return updatedRecognitions;
    }

    public double detectPosition() {
        //BLOCKPOS skystonePos = BLOCKPOS.NONE;
        double center = -100;
        List<Recognition> recognizedList = recognize();
        if (recognizedList != null)
            for (int i = 0; i < recognizedList.size(); i++) {
                telemetry.addData("Label",recognizedList.get(i).getLabel());
                if (recognizedList.get(i).getLabel().equalsIgnoreCase("Skystone")) {
                    center = recognizedList.get(i).getLeft() + recognizedList.get(i).getWidth() / 2;
                }
                telemetry.update();
            }
        //skystonePos = posFinder.findLocation(recognizedList);

        return center;
    }

    public void intake(int pw, int seconds) {
        Thread thread = new Thread() {
            public void run() {
                hwMap.leftIntake.setPower(-pw);
                hwMap.rightIntake.setPower(pw);
                try {
                    Thread.sleep(seconds * 1000);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        };

        thread.start();
    }
}
