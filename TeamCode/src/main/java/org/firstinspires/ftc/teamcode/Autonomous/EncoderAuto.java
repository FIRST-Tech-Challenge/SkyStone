package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Detect;

import java.util.List;

@Autonomous(name = "Autonomous", group = "LinearOpMode")
@Disabled
public class EncoderAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    private static final String LABEL_FIRST_ELEMENT = "skystone";
    private static final String LABEL_SECOND_ELEMENT = "stone";

    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public FieldPosition fieldPosition = null;
    public HardwareMap hwMap;
    public double imgWidth;
    Detect detect;
    private int[] skystonePositions;

    double power = 0.7;

    @Override
    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);
        detect = new Detect();

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE); //???
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!isStarted()) {
            if (gamepad1.a) {
                fieldPosition = FieldPosition.BLUE_FOUNDATION;
            } else if (gamepad1.b) {
                fieldPosition = FieldPosition.RED_FOUNDATION;
            } else if (gamepad1.y) {
                fieldPosition = FieldPosition.RED_QUARY;
            } else if (gamepad1.x) {
                fieldPosition = FieldPosition.BLUE_QUARY;
            }

            if (gamepad1.start) {
                switch (fieldPosition) {
                    case RED_QUARY:
                        drivetrain.resetEncoders();
                        initVuforia();
                        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                            initTfod();
                        } else {
                            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                        }

                        if (tfod != null) {
                            tfod.activate();
                        }

                        List<Recognition> recognizedList = recognize();
                        if (recognizedList != null)
                            skystonePositions = detect.getSkystonePositionsRed(recognizedList, imgWidth);
                        break;
                    case BLUE_QUARY:
                        drivetrain.resetEncoders();
                        initVuforia();

                        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                            initTfod();
                        } else {
                            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                        }
                        if (tfod != null) {
                            tfod.activate();
                        }

                        List<Recognition> recognized = recognize();
                        if (recognized != null)
                            skystonePositions = detect.getSkystonePositionsBlue(recognized, imgWidth);
                        break;
                }
            }
            telemetry.addData("Position", fieldPosition);
            telemetry.update();

        }

        /** Wait for the game to begin */
        waitForStart();
        if (opModeIsActive()) {
            switch (fieldPosition){
                case RED_QUARY:
                    if(skystonePositions[0] == 1){
                        /*drivetrain.odometryStrafe(0.5, 6, false);
                        intake(1, true);
                        drivetrain.encoderDrive(0.5,34);
                        sleep(500);*/
                    } else if(skystonePositions[0] == 2){

                    } else if(skystonePositions[0] == 3){

                    }
                    break;
                case RED_FOUNDATION:
                    break;
                case BLUE_QUARY:
                    break;
                case BLUE_FOUNDATION:
                    break;
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
            imgWidth = frame.getImage(0).getWidth();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.85;
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

    public void intake(int pw, boolean on) {
        Thread thread = new Thread() {
            public void run() {
                if(on) {
                    hwMap.leftIntake.setPower(-pw);
                    hwMap.rightIntake.setPower(pw);
                } else {
                    hwMap.leftIntake.setPower(0);
                    hwMap.rightIntake.setPower(0);
                }
            }
        };
        thread.start();
    }
}
