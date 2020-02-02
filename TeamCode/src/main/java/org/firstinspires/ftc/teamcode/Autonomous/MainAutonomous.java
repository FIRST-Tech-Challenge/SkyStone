package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Autonomous", group = "LinearOpMode")
//@Disabled
public class MainAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    private static final String LABEL_FIRST_ELEMENT = "skystone";
    private static final String LABEL_SECOND_ELEMENT = "stone";

    // TODO : move vuforia key to separate file -- this is pretty i l l e g a l
    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private FieldPosition fieldPosition = null;
    private HardwareMap hwMap;
    private double imgWidth;
    private Detect detect;
    private int[] skystonePositions;
    private Pose2d startingPos;
    private Path path;
    private SampleMecanumDriveBase straightDrive;
    private SampleMecanumDriveBase strafeDrive;
    private boolean initialize = false;
    public BNO055IMU imu;

    private enum CameraController{
        WEBCAM, PHONECAM
    }

    @Override
    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);
        detect = new Detect();

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        // Setup drive, lift, and intake motors
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveConstantsPID.updateConstantsFromProperties();

        while (!isStarted() && !isStopRequested()) {

            // Select starting position from user input
            if (fieldPosition == null) {
                telemetry.addData("SELECT STARTING LOCATION", "Press one of the following buttons below to " +
                        "select the autonomous starting position. Once you have selected, press the \"start\" button " +
                        "on gamepad A.");
                telemetry.addData("A", FieldPosition.BLUE_FOUNDATION_PARK);
                telemetry.addData("B", FieldPosition.RED_FOUNDATION_PARK);
                telemetry.addData("Y", FieldPosition.RED_QUARY);
                telemetry.addData("X", FieldPosition.BLUE_QUARY);
            }

            if(!gamepad1.start) {
                if (gamepad1.a) {
                    fieldPosition = FieldPosition.BLUE_FOUNDATION_PARK;
                    startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));
                } else if (gamepad1.b) {
                    fieldPosition = FieldPosition.RED_FOUNDATION_PARK;
                    startingPos = new Pose2d(new Vector2d(20.736, -63.936), Math.toRadians(90));
                } else if (gamepad1.y) {
                    fieldPosition = FieldPosition.RED_QUARY;
                    startingPos = new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0));
                } else if (gamepad1.x) {
                    fieldPosition = FieldPosition.BLUE_QUARY;
                    startingPos = new Pose2d(new Vector2d(-34.752, 63.936), Math.toRadians(0));
                } else if(gamepad1.right_bumper){
                    fieldPosition = FieldPosition.BLUE_FOUNDATION_DRAG;
                    startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));
                } else if(gamepad1.left_bumper){
                    fieldPosition = FieldPosition.RED_FOUNDATION_DRAG;
                    startingPos = new Pose2d(new Vector2d(20.736, -63.936), Math.toRadians(90));
                }
            }

            if (fieldPosition != null && !initialize)
                telemetry.addData("SELECTED STARTING LOCATION", fieldPosition);

            if (gamepad1.start && fieldPosition != null)
                initialize = true;

            if (initialize) {
                telemetry.addData("STATUS", "Calibrating IMU...");
                telemetry.update();
                if (DriveConstantsPID.USING_BULK_READ == false)
                    straightDrive = new SampleMecanumDriveREV(hardwareMap, false);
                else
                    straightDrive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
                /*
                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true);
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                */
                telemetry.addData("STATUS", "Done!");
                telemetry.update();
                path = new Path(hwMap, this, straightDrive, startingPos, hardwareMap, imu);

                if (fieldPosition == FieldPosition.RED_QUARY || fieldPosition == FieldPosition.BLUE_QUARY) {
                    telemetry.addData("STATUS", "Initializing TensorFlow...");
                    telemetry.update();

                    drivetrain.resetEncoders();

                    //if(fieldPosition == FieldPosition.RED_QUARY)
                        initVuforia(CameraController.WEBCAM);
                    //else if(fieldPosition == FieldPosition.BLUE_QUARY)
                    //    initVuforia(CameraController.PHONECAM);

                    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                        initTfod();
                    } else {
                        telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                    }

                    if (tfod != null) {
                        tfod.activate();
                    }

                    telemetry.addData("STATUS", "Done!");
                    telemetry.update();
                }
                break;
            }
            telemetry.update();
        }

        drivetrain.resetEncoders();

        // begin tfod processing before starting -- use it to ascertain the positions of skystones in quarry
        while (!isStarted() && (fieldPosition == FieldPosition.BLUE_QUARY || fieldPosition == FieldPosition.RED_QUARY) &&
                !isStopRequested()) {
            List<Recognition> recognized = recognize();

            if (recognized != null && fieldPosition == FieldPosition.BLUE_QUARY)
                try {
                    skystonePositions = detect.getSkystonePositionsBlue(recognized, imgWidth);
                } catch (Exception e){ e.printStackTrace(); }
            else if (recognized != null && fieldPosition == FieldPosition.RED_QUARY)
                try{
                    skystonePositions = detect.getSkystonePositionsRed(recognized, imgWidth);
                } catch (Exception e){ e.printStackTrace(); }

            telemetry.addData("SKYSTONE POSITIONS", Arrays.toString(skystonePositions));
            telemetry.addData("External Heading",
                    Math.round(Math.toDegrees(straightDrive.getExternalHeading()) * 1000.0) / 1000.0);
            telemetry.addData("Current (starting) Location", path.getPoseEstimate());
            telemetry.update();
            try {
                Thread.sleep(200);
            } catch (Exception e) {
            }
        }

        if (isStopRequested() && tfod != null) {
            tfod.shutdown();
            tfod = null;
        }

        waitForStart();
        if (tfod != null) {
            RobotLogger.dd("", "to shutdown tensor flow");
            tfod.shutdown();
            tfod = null;
            RobotLogger.dd("", "tensor flow is shutdown");
        }
        if (opModeIsActive() && fieldPosition != null) {
            if(skystonePositions != null) {
                sendData();
                //resetLiftEncoder();
                switch (fieldPosition) {
                    case RED_QUARY:
                        path.RedQuary(skystonePositions);
                        break;
                    case RED_FOUNDATION_PARK:
                        path.RedFoundationPark();
                        break;
                    case BLUE_QUARY:
                        path.BlueQuary(skystonePositions);
                        break;
                    case BLUE_FOUNDATION_PARK:
                        path.BlueFoundationPark();
                        break;
                    case BLUE_FOUNDATION_DRAG:
                        path.BlueFoundationDrag();
                        break;
                    case RED_FOUNDATION_DRAG:
                        break;
                }
            } else {
                while (opModeIsActive()) {
                    telemetry.addData("ERROR", "No SKYSTONE position data received!");
                    telemetry.update();
                }
            }
        } else {
            while (opModeIsActive()) {
                telemetry.addData("ERROR", "Please select parameters with gamepad1 upon initialization!");
                telemetry.update();
            }
        }

    }

    private void initVuforia(CameraController controller) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        if(controller == CameraController.WEBCAM)
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
        tfodParameters.minimumConfidence = 0.67;
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

    private void sendData() {

        // inject imu heading and tfod recognition into currently running path from another thread

        Thread update = new Thread() {
            public void run() {
                while (opModeIsActive() && (tfod != null)) {
                    path.updateTFODData(recognize());
                    path.updateHeading();
                    try {
                        Thread.sleep(500);
                    } catch (Exception e) {
                    }
                    if (isStopRequested() && tfod != null)
                        tfod.shutdown();
                }
            }
        };
        update.start();
    }

    private void resetLiftEncoder() {
        Thread resetEncoderLoop = new Thread() {
            public void run() {
                while (opModeIsActive())
                    if (!hwMap.liftReset.getState()) {
                        hwMap.liftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        try {
                            Thread.sleep(300);
                        } catch (Exception e) {
                        }
                        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
            }
        };
        resetEncoderLoop.start();
    }
}
