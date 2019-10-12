package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.libraries.Constants.GAMEPAD_JOYSTICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_ARM_POS_REST;

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
 * Author: Rahul, Sarvesh, Sachin, Shivani
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

//        robot.setServoPosition(SERVO_INTAKE_HOLDER, SERVO_INTAKE_HOLDER_POS_HOLD);
//        robot.setServoPosition(SERVO_SCORING, SERVO_SCORING_POS_RECEIVE);

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

    public void processLatcher() {
        latcherMotor();
//        latcherServo();
    }

    // Uses gamepad 1 bumpers to control movement
    private void latcherMotor() {
//        if (opMode.gamepad1.right_bumper && !robot.isTouchSensorPressed(TOUCH_LATCHER_BOTTOM)) {
//            // Extend
//            robot.setDcMotorPower(MOTOR_LATCHER, .6f);
//        } else if (opMode.gamepad1.left_bumper && !robot.isTouchSensorPressed(TOUCH_LATCHER_TOP)) {
//            // Retract
//            robot.setDcMotorPower(MOTOR_LATCHER, -.6f);
//        } else {
//            robot.setDcMotorPower(MOTOR_LATCHER, 0);
    }
//}

    // Uses gamepad 1 B to switch between positions
//    private void latcherServo() {
//        if (opMode.gamepad1.b && latcherServoInputDelay.seconds() > .25) {
//            if (robot.getServoPosition(SERVO_LATCHER) == SERVO_LATCHER_POS_LATCHED) {
//                robot.setServoPosition(SERVO_LATCHER, SERVO_LATCHER_POS_REST);
//            } else {
//                robot.setServoPosition(SERVO_LATCHER, SERVO_LATCHER_POS_LATCHED);
//            }
//            latcherServoInputDelay.reset();
//        }
//    }

    public void GrabServoArm() {
        if (opMode.gamepad2.x && servoArmInputDelay.seconds() > .25)
            if (robot.getServoPosition(SERVO_ARM) == SERVO_ARM_POS_REST) {
                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_GRAB);
            } else {
                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_REST);
            }
        latcherServoInputDelay.reset();
    }
}
        // Uses gamepad 1 triggers for movement
//    public void processScoringSlide() {
//        if (opMode.gamepad1.right_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
//            // Extend
//            robot.setDcMotorPower(MOTOR_SCORING_SLIDE, -opMode.gamepad1.right_trigger);
//        } else if (opMode.gamepad1.left_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
//            // Retract
//            robot.setDcMotorPower(MOTOR_SCORING_SLIDE, opMode.gamepad1.left_trigger);
//        } else {
//            robot.setDcMotorPower(MOTOR_SCORING_SLIDE, 0);
//        }
//    }

        // Uses gamepad 1 Y and d-pad up/down
//    public void processScoringServo() {
//        // Preset
//        if (opMode.gamepad1.a) {
//            robot.setServoPosition(SERVO_SCORING, SERVO_SCORING_POS_RECEIVE);
//        } else if (opMode.gamepad1.y) {
//            robot.setServoPosition(SERVO_SCORING, SERVO_SCORING_POS_SCORE);
//        }
//
//        // Manual
//        if (opMode.gamepad1.dpad_up && scoringServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_SCORING, .02f);
//            scoringServoInputDelay.reset();
//        } else if (opMode.gamepad1.dpad_down && scoringServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_SCORING, -.02f);
//            scoringServoInputDelay.reset();
//        }
//    }

        // Uses gamepad 2 triggers to move the intake
//    public void processIntakeSlide() {
//        if (opMode.gamepad2.right_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
//            // Extend
//            robot.setDcMotorPower(MOTOR_INTAKE_SLIDE, -opMode.gamepad2.right_trigger);
//        } else if (opMode.gamepad2.left_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
//            // Retract
//            robot.setDcMotorPower(MOTOR_INTAKE_SLIDE, opMode.gamepad2.left_trigger);
//        } else {
//            robot.setDcMotorPower(MOTOR_INTAKE_SLIDE, 0);
//        }
//    }

        // Uses gamepad 2 bumpers & Y
//    public void processIntake() {
//        if (opMode.gamepad2.right_bumper) {
//            robot.setDcMotorPower(MOTOR_INTAKE, -.75f);
//        } else if (opMode.gamepad2.left_bumper) {
//            robot.setDcMotorPower(MOTOR_INTAKE, .75f);
//        } else if (opMode.gamepad2.y) {
//            robot.setDcMotorPower(MOTOR_INTAKE, 0);
//        }
//    }
//
//    // Uses gamepad 2 d-pad
//        public void processIntakeAngle () {
//        if (opMode.gamepad2.dpad_down) {
//            robot.setServoPosition(SERVO_INTAKE_ANGLE, SERVO_INTAKE_ANGLE_POS_INTAKE);
//        } else if (opMode.gamepad2.dpad_up) {
//            robot.setServoPosition(SERVO_INTAKE_ANGLE, SERVO_INTAKE_ANGLE_POS_CRATER);
//        }
//
//        if (opMode.gamepad2.dpad_left && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_INTAKE_ANGLE, .02f);
//            intakeAngleServoInputDelay.reset();
//        } else if (opMode.gamepad2.dpad_right && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_INTAKE_ANGLE, -.02f);
//            intakeAngleServoInputDelay.reset();
//        }
//    }

//    // Uses gamepad 2 B and X
//    public void processIntakeHolder() {
//        if (opMode.gamepad2.b) {
//            robot.setServoPosition(SERVO_INTAKE_HOLDER, SERVO_INTAKE_HOLDER_POS_HOLD);
//        } else if (opMode.gamepad2.x) {
//            robot.setServoPosition(SERVO_INTAKE_HOLDER, SERVO_INTAKE_HOLDER_POS_DEPOSIT);
//        }
//    }
