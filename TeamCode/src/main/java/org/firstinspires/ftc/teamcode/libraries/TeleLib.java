package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.libraries.Constants.GAMEPAD_JOYSTICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_REST;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_ARM_BOTTOM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_ARM_TOP;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_GRAB;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_REST;

//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_INTAKE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_INTAKE_SLIDE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LATCHER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_SCORING_SLIDE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_ANGLE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_ANGLE_POS_CRATER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_ANGLE_POS_INTAKE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_HOLDER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_HOLDER_POS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE_HOLDER_POS_HOLD;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_LATCHER;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_LATCHER_POS_LATCHED;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_LATCHER_POS_REST;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_POS_RECEIVE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_POS_SCORE;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_LATCHER_BOTTOM;
//import static org.firstinspires.ftc.teamcode.libraries.Constants.TOUCH_LATCHER_TOP;

/*
 * Title: TeleLib
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This will contain the methods for TeleOp, and other TeleOp-related programs.
 */

public class TeleLib {
    private Robot robot;
    private LinearOpMode opMode;

    private ElapsedTime latcherServoInputDelay;
    private ElapsedTime scoringServoInputDelay;
    private ElapsedTime intakeAngleServoInputDelay;
    private ElapsedTime servoArmInputDelay;

    public TeleLib(LinearOpMode opMode) {
        robot = new Robot(opMode);
        this.opMode = opMode;

        opMode.gamepad1.setJoystickDeadzone(GAMEPAD_JOYSTICK_TOLERANCE);
        opMode.gamepad2.setJoystickDeadzone(GAMEPAD_JOYSTICK_TOLERANCE);


        latcherServoInputDelay = new ElapsedTime();
        scoringServoInputDelay = new ElapsedTime();
        intakeAngleServoInputDelay = new ElapsedTime();
    }

    public void processDrive() {
        // Values need to be reversed (up on joystick is -1)
        double r = Math.hypot(opMode.gamepad1.left_stick_x, -opMode.gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = opMode.gamepad1.right_stick_x;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, (float) (r * Math.cos(robotAngle) - rightX));
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, (float) (r * Math.sin(robotAngle) + rightX));
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, (float) (r * Math.sin(robotAngle) - rightX));
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, (float) (r * Math.cos(robotAngle) + rightX));
    }

    // Uses gamepad 1 bumpers to control movement
//    private void latcherMotor() {
//        if (opMode.gamepad1.right_bumper && !robot.isTouchSensorPressed(TOUCH_LATCHER_BOTTOM)) {
//            // Extend
//            robot.setDcMotorPower(MOTOR_LATCHER, .6f);
//        } else if (opMode.gamepad1.left_bumper && !robot.isTouchSensorPressed(TOUCH_LATCHER_TOP)) {
//            // Retract
//            robot.setDcMotorPower(MOTOR_LATCHER, -.6f);
//        } else {
//            robot.setDcMotorPower(MOTOR_LATCHER, 0);
//    }

    public void processIntakeMinerals() {
        if (opMode.gamepad1.x) {
            robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, -.5f);
            robot.setDcMotorPower(MOTOR_LEFT_INTAKE, .5f);
        } else if (opMode.gamepad1.y) {
            robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, 0);
            robot.setDcMotorPower(MOTOR_LEFT_INTAKE, 0);
        }
    }

    public void processMoveArmUp() {
        if (opMode.gamepad1.right_bumper && !robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM)) {
            // Extend
            robot.setDcMotorPower(MOTOR_ARM, .4f);
        } else if (opMode.gamepad1.left_bumper && !robot.isTouchSensorPressed(TOUCH_ARM_TOP)) {
            // Retract
            robot.setDcMotorPower(MOTOR_ARM, -.4f);
        } else {
            robot.setDcMotorPower(MOTOR_ARM, 0);
        }
    }

    //}
    public void processServoArm() {
        if (opMode.gamepad2.x && servoArmInputDelay.seconds() > .25)
            if (robot.getServoPosition(SERVO_ARM) == SERVO_ARM_POS_REST) {
                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_GRAB);
            } else {
                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_REST);
            }
        latcherServoInputDelay.reset();
        if (opMode.gamepad1.dpad_up && scoringServoInputDelay.seconds() > .2f) {
            robot.setDeltaServoPosition(SERVO_ARM, .02f);
            scoringServoInputDelay.reset();
        } else if (opMode.gamepad1.dpad_down && scoringServoInputDelay.seconds() > .2f) {
            robot.setDeltaServoPosition(SERVO_ARM, -.02f);
            scoringServoInputDelay.reset();
        }
    }
}


