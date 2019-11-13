package org.firstinspires.ftc.teamcode.libraries;

/*
 * Title: Constants
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This will contain all of the constants we will use in any of our programs.
 */

public class Constants {

    //********** Gamepad Tolerance Constants **********//
    static final float GAMEPAD_JOYSTICK_TOLERANCE = .05f;
    static final float GAMEPAD_TRIGGER_TOLERANCE = .05f;

    //********** DcMotor Indexes **********//
    static final int MOTOR_FRONT_LEFT_WHEEL = 0;
    static final int MOTOR_FRONT_RIGHT_WHEEL = 1;
    static final int MOTOR_BACK_LEFT_WHEEL = 2;
    static final int MOTOR_BACK_RIGHT_WHEEL = 3;
    static final int MOTOR_RIGHT_INTAKE = 4;
    static final int MOTOR_LEFT_INTAKE = 5;
    static final int MOTOR_ARM = 6;

    //********** Servo Indexes **********//
    static final int SERVO_ARM = 0;
    static final int SERVO_GRABBER = 1;
    static final int SERVO_FOUNDATION1 = 2;
    static final int SERVO_FOUNDATION2 = 3;

    //********** Servo Positions **********//
    static final float SERVO_ARM_POS_RECIEVE = .97f;
    static final float SERVO_ARM_POS_SCORE = .27f;
    static final float SERVO_GRABBER_GRAB = .1f;
    static final float SERVO_GRABBER_REST = .44f;
    static final float SERVO_FOUNDATION_GRAB1 = .96f;
    static final float SERVO_FOUNDATION_REST1 = .33f;
    static final float SERVO_FOUNDATION_GRAB2 =.91f;
    static final float SERVO_FOUNDATION_REST2 = .65f;


    //********** Touch Sensor Indexes **********//
    static final int TOUCH_ARM_BOTTOM = 0;
    static final int TOUCH_ARM_TOP = 1;

    //********** CalcMove Constants **********//
    static final float WHEEL_DIAMETER = 9f; // Centimeters
    static final float WHEEL_GEAR_RATIO = (2f / 2);
    static final float NEVEREST_40_REVOLUTION_ENCODER_COUNT = 1120f;
    static final float TRACK_DISTANCE = 30f;

    public enum Direction {FORWARD, BACKWARD, LEFT, RIGHT}

    //********** TensorFlow **********//
    static final float TENSOR_READING_TIME = 3f;

    public enum GoldObjectPosition {LEFT, CENTER, RIGHT}
}