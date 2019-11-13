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
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.NEVEREST_40_REVOLUTION_ENCODER_COUNT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_RECIEVE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_SCORE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER_REST;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TENSOR_READING_TIME;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_ARM_BOTTOM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_ARM_TOP;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_GEAR_RATIO;
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

    }


    //********** Base Motor Methods **********//

    public void calcMove(float centimeters, float power, Constants.Direction direction) {
        // Calculates target encoder position
        final int targetPosition = (int) ((((centimeters / (Math.PI * WHEEL_DIAMETER)) *
                NEVEREST_40_REVOLUTION_ENCODER_COUNT)) * WHEEL_GEAR_RATIO);

        switch (direction) {
            case BACKWARD:
                prepMotorsForCalcMove(targetPosition, targetPosition, targetPosition, targetPosition);
                break;
            case FORWARD:
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
            opMode.idle();
        }

        setBaseMotorPowers(0);
    }

    public void calcMoveIntake(float centimeters, float power, Constants.Direction direction) {
        // Calculates target encoder position
        final int targetPosition = (int) ((((centimeters / (Math.PI * WHEEL_DIAMETER)) *
                NEVEREST_40_REVOLUTION_ENCODER_COUNT)) * WHEEL_GEAR_RATIO);

//        robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, -.5f);
//        robot.setDcMotorPower(MOTOR_LEFT_INTAKE, .5f);
        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);
        switch (direction) {
            case BACKWARD:
                prepMotorsForCalcMove(targetPosition, targetPosition, targetPosition, targetPosition);
                break;
            case FORWARD:
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
            opMode.idle();
        }

        setBaseMotorPowers(0);
        robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, 0);
        robot.setDcMotorPower(MOTOR_LEFT_INTAKE, 0);

    }

    public void calcTurn(int degrees, float power) {
        // Calculates target encoder position
        int targetPosition = (int) (2 * ((TRACK_DISTANCE) * degrees
                * NEVEREST_40_REVOLUTION_ENCODER_COUNT) /
                (WHEEL_DIAMETER * 360));


        prepMotorsForCalcMove(-targetPosition, targetPosition, -targetPosition, targetPosition);

        setBaseMotorPowers(power);

        while (areBaseMotorsBusy()) {
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

        robot.setDcMotorTargetPosition(MOTOR_FRONT_LEFT_WHEEL, frontLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_FRONT_RIGHT_WHEEL, frontRightTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_LEFT_WHEEL, backLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_RIGHT_WHEEL, backRightTargetPosition);

        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);

        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, RUN_TO_POSITION);
    }

    private boolean areBaseMotorsBusy() {
        return robot.isMotorBusy(MOTOR_FRONT_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_FRONT_RIGHT_WHEEL) ||
                robot.isMotorBusy(MOTOR_BACK_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_BACK_RIGHT_WHEEL);
    }


    //********** Motor Methods **********//

    public void intakeStone() {
        ElapsedTime time = new ElapsedTime();

        robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, -.5f);
        robot.setDcMotorPower(MOTOR_LEFT_INTAKE, .5f);
        while (time.seconds() <= 5) {
            opMode.idle();
        }
    }

    public void moveArmDownScoreServoArmGrab() throws InterruptedException {

        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_SCORE);

        Thread.sleep(500);

        robot.setDcMotorPower(MOTOR_ARM, 0.5f);

        while (!robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM)) {
            opMode.idle();
            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM));
            opMode.telemetry.update();
        }
        opMode.telemetry.addData("Status", "Pressed");
        opMode.telemetry.update();

        robot.setDcMotorPower(MOTOR_ARM, 0);

        Thread.sleep(1000);
        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
    }

    public void moveArmUp() {
        robot.setDcMotorPower(MOTOR_ARM, -0.5f);

        while (!robot.isTouchSensorPressed(TOUCH_ARM_TOP)) {
            opMode.idle();
            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_ARM_TOP));
            opMode.telemetry.update();
        }
    }

    public void distanceSensorMove() {
        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, .4f);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, .4f);
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, .4f);
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, .4f);

        while (robot.getWallDistanceCenti() >= 45) {
            opMode.idle();
        }
    }

    public void moveArmDown() {
        robot.setDcMotorPower(MOTOR_ARM, 0.7f);

        while (!robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM)) {
            opMode.idle();
            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM));
            opMode.telemetry.update();
        }
    }

    public void moveArmUpSeconds()  {
        ElapsedTime time = new ElapsedTime();

        robot.setDcMotorPower(MOTOR_ARM, -.8f);
        while (time.seconds() <= 1.75) {
            opMode.idle();
        }
        robot.setDcMotorPower(MOTOR_ARM, 0);
    }
    public void moveArmDownSeconds() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();

        robot.setDcMotorPower(MOTOR_ARM, .7f);
        while (time.seconds() <= 1.25) {
            opMode.idle();
        }
        robot.setDcMotorPower(MOTOR_ARM, 0);
    }

    //********** Servo Methods **********//

    public void recieveServoArm() {
        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_RECIEVE);
    }

    public void scoreServoArm() {
        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_SCORE);
    }

    public void grabServo() {
        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
    }

    public void scoreServo() {
        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);
    }

    public void latchServoFoundation() {
        robot.setServoPosition(SERVO_FOUNDATION1, -SERVO_FOUNDATION_GRAB1);
        robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_GRAB2);
    }

    public void restServoFoundation() {
        robot.setServoPosition(SERVO_FOUNDATION1, SERVO_FOUNDATION_REST1);
        robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_REST2);
    }

    //********** Tensor Flow Methods **********//

    private void initTfod() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // parameters.vuforiaLicenseKey = VUFORIA_KEY;
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