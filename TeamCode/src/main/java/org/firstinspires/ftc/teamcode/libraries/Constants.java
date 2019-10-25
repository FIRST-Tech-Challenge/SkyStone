package org.firstinspires.ftc.teamcode.libraries;

/*
 * Title: Constants
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Rahul, Poorvi, Varnika, Sarvesh, Sachin, Shivani
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
//    static final int MOTOR_LATCHER = 4;
//    static final int MOTOR_SCORING_SLIDE = 5;
//    static final int MOTOR_INTAKE_SLIDE = 6;
//    static final int MOTOR_INTAKE = 7;

    //********** Servo Indexes **********//
//    static final int SERVO_LATCHER = 0;
//    static final int SERVO_INTAKE_ANGLE = 1;
//    static final int SERVO_SCORING = 2;
//    static final int SERVO_INTAKE_HOLDER = 3;
//    static final int SERVO_ARM = 0;

    //********** Servo Positions **********//
//    static final float SERVO_LATCHER_POS_LATCHED = .64f;
//    static final float SERVO_LATCHER_POS_REST = .38f;
//    static final float SERVO_SCORING_POS_RECEIVE = .5f;
//    static final float SERVO_SCORING_POS_SCORE = .82f;
//    static final float SERVO_INTAKE_ANGLE_POS_INTAKE = .12f;
//    static final float SERVO_INTAKE_ANGLE_POS_CRATER = .47f;
//    static final float SERVO_INTAKE_HOLDER_POS_HOLD = .62f;
//    static final float SERVO_INTAKE_HOLDER_POS_DEPOSIT = .17f;
//    static final float SERVO_SCORING_POS_MARKER_DEP = .87f;
//    static final float SERVO_SCORING_POS_RETRACT_MARKER = .65f;
//    static final float SERVO_ARM_POS_GRAB = .35f;
//    static final float SERVO_ARM_POS_REST = .75f;

    //********** Touch Sensor Indexes **********//
//    static final int TOUCH_LATCHER_TOP = 0;
//    static final int TOUCH_LATCHER_BOTTOM = 1;

    //********** CalcMove Constants **********//
    static final float WHEEL_DIAMETER = 9f; // Centimeters
    static final float WHEEL_GEAR_RATIO = (2f / 2);
    static final float NEVEREST_40_REVOLUTION_ENCODER_COUNT = 1120f;
    static final float TRACK_DISTANCE = 30f;

    public enum Direction {FORWARD, BACKWARD, LEFT, RIGHT}

    //********** TensorFlow **********//
//     Vuforia Key compatible with external camera
   // static final String VUFORIA_KEY = "ARSzhHP/////AAABmQ3dyIKKfkcipjZh0HtnoDEkjuCn18CTNUWRN7PTFoedxZLS+QZmpkyXpQnQXFpQ5ol//l0ZwTejVrGRQ4i/kQBrrFJ8E0C7ckr4lzf5bLCvi1/E9x8anPwt2D0UToZ3MB5jPx4T6s/EOs575BtxjL7uv5jrCbQDsXebm2PROU4zC/Dj7+AYFkKCqD3YYLbGPGV4YoSgp9Ihoe+ZF/eae0FLG8K/o4eyfZj0B3aXkRvYi3dC5LY+c76aU72bKTrQ2PDYSxDG8xCaY1JyEyfDA6XqjHjYMvh0BBbb8bAQvPgG6/G50+5L+c/a8u6sbYJLbvVtXdMtrG1EA4CglbnsDs7GyyJmH5AusSwIDb9DQnTA";
    static final float TENSOR_READING_TIME = 3f;

    public enum GoldObjectPosition {LEFT, CENTER, RIGHT}
}