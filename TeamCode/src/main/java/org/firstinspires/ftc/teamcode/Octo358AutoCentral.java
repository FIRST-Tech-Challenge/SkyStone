package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Contains setup for all basic drive train actions, Vuforia, TFOD used in Autonomous regions
 */

abstract class Octo358AutoCentral extends LinearOpMode {

    /**
     * Configure chassis & drive train settings
     */
    //TODO: Configure chassis & drive train settings
    private static final double DISTANCE_COMPENSATION = (double) 4 / 3;
    private static final double WHEEL_DIAMETER = (double) 4;
    private static final double DRIVE_TRAIN_LENGTH = (double) 16;
    private static final double DRIVE_TRAIN_WIDTH = (double) 16;
    //Andy Mark Motors have 1120 ticks per revolution
    //  Tetrix Motors  have 1440 ticks per revolution
    private static final int ENCODER_TICKS = 1120;

    //Declare Motors
    private DcMotor fL = hardwareMap.dcMotor.get("fL"); //0
    private DcMotor fR = hardwareMap.dcMotor.get("fR"); //1
    private DcMotor bL = hardwareMap.dcMotor.get("bL"); //2
    private DcMotor bR = hardwareMap.dcMotor.get("bR"); //3

    /**
     * SkyStone Robot, Vuforia, and TenserFlow Object Detection settings
     */
    //TODO: Finish TFOD & Vuforia Navigation tracking methods
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AXzW9CD/////AAAAGTPAtr9HRUXZmowtd9p0AUwuXiBVONS/c5x1"
            + "q8OvjMrQ8/XJGxEp0TP9Kl8PvqSzeXOWIvVa3AeB6MyAQboyW/Pgd/c4a4U/VBs1ouUsVBkEdbaq1iY7RR0c"
            + "jYr3eLwEt6tmI37Ugbwrd5gmxYvOBQkGqzpbg2U2bVLycc5PkOixu7PqPqaINGZYSlvUzEMAenLOCxZFpsay"
            + "uCPRbWz6Z9UJfLeAbfAPmmDYoKNXRFll8/jp5Ie7iAhSQgfFggWwyiqMRCFA3GPTsOJS4H1tSiGlMjVzbJnk"
            + "usPKXfJ0dK3OH9u7ox9ESpi91T0MemXw3nn+/6QRvjGtgFH+wMDuQX7ta89+yW+wqdXX9ZQu8BzY";
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    // the height of the center of the target image above the floor
    private static final float mmTargetHeight = (6) * mmPerInch;
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the
    // physical dimensions.
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59; // Units are degrees
    private static final float bridgeRotZ = 180;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    //vuforia is the variable we will use to store our instance of the Vuforia localization engine.
    private VuforiaLocalizer vuforia;

    //tfod is the variable we will use to store our instance of the TensorFlow Object Detection
    //engine.
    private TFObjectDetector tfod;

    /**
     * Drive Train methods (same definition as in TeleOp, with right-hand positive rotation rules)
     */
    void drive(double power, double distance) {
        //Distance measured in inches
        int ticks = distanceToTicks(distance);

        driveTrain(power, ticks, ticks, ticks, ticks, 0.9);
    }

    void strafeLeft(double power, double distance) {
        //Distance measured in inches
        int ticks = distanceToTicks(distance);

        driveTrain(power, -ticks, ticks, ticks, -ticks, 0.95);
    }

    void rotate(double power, double degree) {
        //Distance measured in inches and angles in degrees
        int ticks = degreeToTicks(degree, false);
        driveTrain(power, -ticks, ticks, -ticks, ticks);
    }

    void turnTo(double power, double degree) {
        //Distance measured in inches and angles in degrees
        int ticks = degreeToTicks(degree, true);
        driveTrain(power, -ticks, ticks, -ticks, ticks);
    }

    /**
     * Drive Train core method
     */
    void driveTrain(double power, int fLTicks, int fRTicks, int bLTicks, int bRTicks,
            double shiftAt) {

        int i = (abs(fLTicks) + abs(fRTicks) + abs(bLTicks) + abs(bRTicks)) / 4;
        if (i * shiftAt > 3360) {
            shiftAt = 3360 / i;
        }

        int fLOriginalPosition = fL.getCurrentPosition();
        int fROriginalPosition = fR.getCurrentPosition();
        int bLOriginalPosition = bL.getCurrentPosition();
        int bROriginalPosition = bR.getCurrentPosition();

        fL.setTargetPosition(fLOriginalPosition + fLTicks);
        fR.setTargetPosition(fROriginalPosition + fRTicks);
        bL.setTargetPosition(bLOriginalPosition + bLTicks);
        bR.setTargetPosition(bROriginalPosition + bRTicks);

        setRunModeRunToPosition();

        //Sign of power does not matter with RunToPosition
        setPower(power);

        //Default (-1)
        if (shiftAt == -1) {
            while (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()) {
            }
        } else {
            while (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()) {
                if (fL.getCurrentPosition() - fLOriginalPosition < shiftAt * fLTicks &&
                        fR.getCurrentPosition() - fROriginalPosition < shiftAt * fRTicks &&
                        bL.getCurrentPosition() - bLOriginalPosition < shiftAt * bLTicks &&
                        bR.getCurrentPosition() - bROriginalPosition < shiftAt * bRTicks) {
                    fL.setPower(((fL.getCurrentPosition() - fLOriginalPosition + 1) / (shiftAt
                            * fLTicks)) * power);
                    fR.setPower(((fR.getCurrentPosition() - fROriginalPosition + 1) / (shiftAt
                            * fRTicks)) * power);
                    bL.setPower(((bL.getCurrentPosition() - bLOriginalPosition + 1) / (shiftAt
                            * bLTicks)) * power);
                    bR.setPower(((bR.getCurrentPosition() - bROriginalPosition + 1) / (shiftAt
                            * bRTicks)) * power);
                }
                if (fL.getCurrentPosition() - fLOriginalPosition > (1 - shiftAt) * fLTicks &&
                        fR.getCurrentPosition() - fROriginalPosition > (1 - shiftAt) * fRTicks &&
                        bL.getCurrentPosition() - bLOriginalPosition > (1 - shiftAt) * bLTicks &&
                        bR.getCurrentPosition() - bROriginalPosition > (1 - shiftAt) * bRTicks) {
                    //setPower(shiftRatio * power); //Legacy
                    fL.setPower(((fLOriginalPosition + fLTicks - fL.getCurrentPosition() + 1) /
                            (shiftAt * fLTicks)) * power);
                    fR.setPower(((fROriginalPosition + fRTicks - fR.getCurrentPosition() + 1) /
                            (shiftAt * fRTicks)) * power);
                    bL.setPower(((bLOriginalPosition + bLTicks - bL.getCurrentPosition() + 1) /
                            (shiftAt * bLTicks)) * power);
                    bR.setPower(((bROriginalPosition + bRTicks - bR.getCurrentPosition() + 1) /
                            (shiftAt * bRTicks)) * power);
                }
            }
        }

        setPower(0);
        setRunModeRunUsingEncoders();
    }

    void driveTrain(double power, int fLTicks, int fRTicks, int bLTicks, int bRTicks) {
        driveTrain(power, fLTicks, fRTicks, bLTicks, bRTicks, -1);
    }

    /**
     * Encoder Settings
     */
    void setRunModeRunUsingEncoders() {
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setRunModeRunToPosition() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    private void ResetEncoders() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    */

    /**
     * Motor Configurations
     */
    void configureMotors() {
        //Motor Settings
        fL.setDirection(DcMotor.Direction.REVERSE);                //0
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //1

        bL.setDirection(DcMotor.Direction.REVERSE);                //2
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //3
    }

    void setPower(double power) {
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
    }

    /**
     * Motor Ticks calculation (all lengths measured in inches)
     */
    int distanceToTicks(double distance) {
        return (int) (
                ((distance / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS)) * DISTANCE_COMPENSATION
                        + 0.5);
    }

    int degreeToTicks(double degree, boolean optimize) {
        if (optimize) {
            degree = degree / 360;
            if (degree > 180) {
                degree = degree - 360;
            } else if (degree < -180) {
                degree = 360 + degree;
            }
        }
        double distance = (degree / 360) * (sqrt(DRIVE_TRAIN_WIDTH * DRIVE_TRAIN_WIDTH +
                DRIVE_TRAIN_LENGTH * DRIVE_TRAIN_LENGTH) * PI);
        return distanceToTicks(distance);
    }

    /**
     * Vuforia & TensorFlow Object Detection methods
     */
    //TODO: Finish TFOD & Vuforia Navigation tracking methods
    //Initialize the Vuforia localization engine.
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    //Initialize the TensorFlow Object Detection engine.
    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(
                tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    void startTfod() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();
        }
    }

    void tfodOff() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
