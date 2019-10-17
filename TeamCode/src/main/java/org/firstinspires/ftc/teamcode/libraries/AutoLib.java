package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_INTAKE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_INTAKE_SLIDE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LATCHER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_SCORING_SLIDE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.NEVEREST_40_REVOLUTION_ENCODER_COUNT;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_ANGLE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_ANGLE_POS_CRATER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_LATCHER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_LATCHER_POS_REST;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_POS_MARKER_DEP;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_POS_RETRACT_MARKER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_LATCHER_BOTTOM;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_LATCHER_TOP;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_REST;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TENSOR_READING_TIME;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_GEAR_RATIO;

import static org.firstinspires.ftc.teamcode.libraries.Constants.TENSOR_READING_TIME;
import static org.firstinspires.ftc.teamcode.libraries.Constants.VUFORIA_KEY;

/*
 * Title: AutoLib
 * Date Created: 10/28/2018
 * Date Modified: 1/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Library
 * Description: This will contain the methods for Autonomous, and other autonomous-related programs.
 */

public class AutoLib {
    private Robot robot;
    private LinearOpMode opMode;

    // Declaring TensorFlow detection
    private TFObjectDetector tfod;

    //
    public AutoLib(LinearOpMode opMode) {
        robot = new Robot(opMode);
        this.opMode = opMode;

        // initTfod();
    }


    //********** Base Motor Methods **********//

    public void calcMove(float centimeters, float power, Constants.Direction direction) {
        // Calculates target encoder position
        final int targetPosition = (int) ((((centimeters / (Math.PI * WHEEL_DIAMETER)) *
                NEVEREST_40_REVOLUTION_ENCODER_COUNT)) * WHEEL_GEAR_RATIO);

        switch (direction) {
            case FORWARD:
                prepMotorsForCalcMove(targetPosition, targetPosition, targetPosition, targetPosition);
                break;
            case BACKWARD:
                prepMotorsForCalcMove(-targetPosition, -targetPosition, -targetPosition, -targetPosition);
                break;
            case LEFT:
                prepMotorsForCalcMove(-targetPosition, targetPosition, targetPosition, -targetPosition);
                break;
            case RIGHT:
                prepMotorsForCalcMove(targetPosition, -targetPosition, -targetPosition, targetPosition);
        }

        setBaseMotorPowers(power);

        while (areBaseMotorsBusy()) {
//            opMode.telemetry.addData("Left", robot.getDcMotorPosition(LEFT_WHEEL));
//            opMode.telemetry.addData("Right", robot.getDcMotorPosition(RIGHT_WHEEL));
//            opMode.telemetry.update();
            opMode.idle();
        }

        setBaseMotorPowers(0);      //TODO: Might need to uncomment
    }

    public void calcTurn(int degrees, float power) {
        // Calculates target encoder position
        int targetPosition = (int) (2 * ((TRACK_DISTANCE) * degrees
                * NEVEREST_40_REVOLUTION_ENCODER_COUNT) /
                (WHEEL_DIAMETER * 360));


        prepMotorsForCalcMove(-targetPosition, targetPosition, -targetPosition, targetPosition);    //TODO: Check negative signs

        setBaseMotorPowers(power);

        while (areBaseMotorsBusy()) {
//            opMode.telemetry.addData("Left", robot.getDcMotorPosition(LEFT_WHEEL));
//            opMode.telemetry.addData("Right", robot.getDcMotorPosition(RIGHT_WHEEL));
//            opMode.telemetry.update();
            opMode.idle();
        }

        setBaseMotorPowers(0);
    }

    private void setBaseMotorPowers(float power) {
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, power);
    }

    private void prepMotorsForCalcMove(int frontLeftTargetPosition, int frontRightTargetPosition,
                                       int backLeftTargetPosition, int backRightTargetPosition) {
        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);

        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, RUN_TO_POSITION);

        robot.setDcMotorTargetPosition(MOTOR_FRONT_LEFT_WHEEL, frontLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_FRONT_RIGHT_WHEEL, frontRightTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_LEFT_WHEEL, backLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_RIGHT_WHEEL, backRightTargetPosition);
    }

//    public void moveLinearSlideToDepot(int encoderCount) {
//        robot.setDcMotorMode(MOTOR_INTAKE_SLIDE, STOP_AND_RESET_ENCODER);
//        robot.setDcMotorMode(MOTOR_INTAKE_SLIDE, RUN_TO_POSITION);
//        robot.setDcMotorTargetPosition(MOTOR_INTAKE_SLIDE, encoderCount);
//
//        robot.setDcMotorPower(MOTOR_INTAKE_SLIDE, .1f);
//
//        while (robot.isMotorBusy(MOTOR_INTAKE_SLIDE)) {
//            opMode.telemetry.addData("Intake count", robot.getDcMotorPosition(MOTOR_INTAKE_SLIDE));
//            opMode.telemetry.update();
//            opMode.idle();
//        }
//
//        robot.setDcMotorPower(MOTOR_INTAKE_SLIDE, 0);
//    }

    private boolean areBaseMotorsBusy() {
        return robot.isMotorBusy(MOTOR_FRONT_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_FRONT_RIGHT_WHEEL) ||
                robot.isMotorBusy(MOTOR_BACK_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_BACK_RIGHT_WHEEL);
    }

    public void moveArm() {
        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_GRAB);
    }

//    public void intakeMinerals() {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_INTAKE, .5f);
//        while (time.seconds() <= 3) {
//            opMode.idle();
//        }
//    }

//    public void depositMarker() {
//        robot.setServoPosition(SERVO_SCORING, SERVO_SCORING_POS_MARKER_DEP);
//    }
//
//    public void retractDeposit() {
//        robot.setServoPosition(SERVO_SCORING, SERVO_SCORING_POS_RETRACT_MARKER);
//
//    }
//
//    public void moveScoringArm() {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_SCORING_SLIDE, -.6f);
//        while (time.seconds() <= .6) {
//            opMode.idle();
//        }
//        robot.setDcMotorPower(MOTOR_SCORING_SLIDE,0f);
//
//    }
//
//    public void stopintake() {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_INTAKE, 0);
//    }

    //********** Latcher Methods **********//

//    public void landOnGround() throws InterruptedException {
//        robot.setDcMotorPower(MOTOR_LATCHER, -0.7f);
//        // The motor will stop when it detects that it's on the ground
//        while (!robot.isTouchSensorPressed(TOUCH_LATCHER_TOP)) {
//            //opMode.idle();
////            opMode.telemetry.addData("Status",  robot.isTouchSensorPressed(TOUCH_LATCHER_BOTTOM));
////            opMode.telemetry.update();
//
//        }
////        opMode.telemetry.addData("Status", "Pressed");
////        opMode.telemetry.update();
//
//        robot.setDcMotorPower(MOTOR_LATCHER, 0);
//
//        robot.setServoPosition(SERVO_LATCHER, SERVO_LATCHER_POS_REST);
//    }
//
//    public void moveLatcherToBottom() {
//        robot.setDcMotorPower(MOTOR_LATCHER, .6f);
//        while (!robot.isTouchSensorPressed(TOUCH_LATCHER_BOTTOM)) {
//            opMode.idle();
//            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_LATCHER_BOTTOM));
//            opMode.telemetry.update();
//
//        }
//        robot.setDcMotorPower(MOTOR_LATCHER, 0);
//    }
//
//    //********** Servo Methods **********//
//
//    public void setPositionintakeMinerals() {
//        robot.setServoPosition(SERVO_INTAKE_ANGLE, SERVO_INTAKE_ANGLE_POS_CRATER);
//    }


    //********** Tensor Flow Methods **********//

    private void initTfod() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        /*
         * Configure Tensor Flow
         */
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public Constants.GoldObjectPosition readGoldObjectPosition() {
        if (tfod != null) {
            tfod.activate();
        }

        Constants.GoldObjectPosition goldObjectPosition = null;
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (time.seconds() < TENSOR_READING_TIME) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineralX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineralX == -1) {
                            silverMineralX = (int) recognition.getLeft();
                        }

                        if (goldMineralX != -1 && silverMineralX != -1) {
                            if (goldMineralX < silverMineralX) {
                                goldObjectPosition = Constants.GoldObjectPosition.CENTER;
                            } else if (goldMineralX > silverMineralX) {
                                goldObjectPosition = Constants.GoldObjectPosition.RIGHT;
                            }
                        } else if (goldMineralX == -1 && silverMineralX != 1) {
                            goldObjectPosition = Constants.GoldObjectPosition.LEFT;
                        }
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }

        return goldObjectPosition;
    }
}