package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.NEVEREST_40_REVOLUTION_ENCODER_COUNT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_GEAR_RATIO;

//@TeleOp(name="SKYSTONE Vuforia Nav Webcam1", group ="Concept")
public class AutoLib2019 extends ConceptVuforiaSkyStoneNavigationWebcam2{
    private Robot robot;
    private LinearOpMode opMode;


    public AutoLib2019(LinearOpMode opMode) {
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

    private boolean areBaseMotorsBusy() {
        return robot.isMotorBusy(MOTOR_FRONT_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_FRONT_RIGHT_WHEEL) ||
                robot.isMotorBusy(MOTOR_BACK_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_BACK_RIGHT_WHEEL);
    }
    public void moveArm() {
        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_GRAB);
    }


    public String findSkyStone() {
        String positionSkystone = "";
        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if(trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Stone target is visible");
                    }
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).

            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                double xPosition = translation.get(0);
                if (xPosition < -5) {
                    positionSkystone = "Right";
                } else {
                    positionSkystone = "Center";
                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                positionSkystone = "Left";
                telemetry.addData("Visible Target", "none");
            }
            telemetry.addData("Skystone Position", positionSkystone);
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        return positionSkystone;
    }

}
