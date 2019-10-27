package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Red Side", group = "Concept")
//@Disabled
public class TFConcept extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    
    HardwareRobot robot = new HardwareRobot();
    private BNO055IMU imu;
    double lastZ = 0;
    int turns = 0;

    Servo leftArm = null;
    Servo rightArm = null;

    double MID_SERVO = 0.5;
    double ARM_UP_POWER = 0.45;
    double ARM_DOWN_POWER = -0.45;

    
    private static final String VUFORIA_KEY =
            "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

  public void robotInit() {
        robot.init(hardwareMap);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        leftArm = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");
        leftArm.setPosition(-0.5);
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        robotInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List < Recognition > updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;
                        for (Recognition recognition: updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                            // HERE IS WHERE THE POSITIONING CODE WILL GO
                        if (recognition.getLabel() == "Skystone"){
                            double midH = (recognition.getLeft()+recognition.getRight())/2;
                            double midV = (recognition.getTop()+recognition.getBottom())/2;
                            
                           // if (100 < midH && midH < 150 && 75 < midV && midV < 125) {
                           if (100 <= midH && midH <= 150 && 50 <= midV && midV <= 150) {
                                telemetry.addData("SkyStone", "Left");
                                driveYBT(0, 0, 0.2, .75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, 1.75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.3, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.2, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.2, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, .5);

                                driveYBT(0, 0, 0, 0.5);
                                moveYW(0, 0, 0);
                            } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 150) {
                                telemetry.addData("SkyStone", "Center");
                                driveYBT(0, 0, -0.3, .25);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, 1.75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.3, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.2, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.2, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, .5);

                                driveYBT(0, 0, 0, 0.5);
                                moveYW(0, 0, 0);
                            }
                            else if (400 <= midH && midH <= 550 && 50 <= midV && midV <= 150) {
                                telemetry.addData("SkyStone", "Right");
                                driveYBT(0, 0, -0.3, .75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, .75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.3, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.2, 2);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 1.9);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 0.75);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.2, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, 0.375);

                                driveYBT(0, 0, 0, 0.5);
                                moveYW(0, 0, 0);
                            }
                            }
                        }

                        
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
 public void moveYW(double forward, double turn, double strafe) {
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);
    }

    public void moveYB(double forward, double bearing, double strafe) {
        Orientation angles = imu.getAngularOrientation();
        moveYW(forward, (heading() - bearing) * 0.01, strafe);
    }

    public void driveYBT(double forward, double bearing, double strafe, double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.seconds() > sec) break;
            moveYB(forward, bearing, strafe);
        }
    }

    public double heading() {
        Orientation angles = imu.getAngularOrientation();
        double imuZ = angles.firstAngle;
        // see if cross boundary from plus to minus or vice-versa
        if (lastZ > 140 && imuZ < -140) turns++;
        if (lastZ < -140 && imuZ > 140) turns--;
        lastZ = imuZ;
        return imuZ + turns * 360;
    }
}
