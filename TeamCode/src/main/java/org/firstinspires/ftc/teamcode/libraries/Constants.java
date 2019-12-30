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
    static final int MOTOR_ARM_LEFT = 6;
    static final int MOTOR_ARM_RIGHT = 7;

    //********** Servo Indexes **********//
    static final int SERVO_ARM = 0;
    static final int SERVO_GRABBER = 1;
    static final int SERVO_FOUNDATION1 = 2;
    static final int SERVO_FOUNDATION2 = 3;
    static final int SERVO_SCORING_ARM = 4;
    static final int SERVO_AUTONOMOUS_GRABBER = 5;
    static final int SERVO_AUTONOMOUS_ARM = 6;

    //********** Servo Positions **********//
    static final float SERVO_ARM_POS_RECIEVE = .97f;
    static final float SERVO_ARM_POS_SCORE = .27f;
    static final float SERVO_GRABBER_GRAB = 0f;
    static final float SERVO_GRABBER_REST = .44f;
    static final float SERVO_FOUNDATION_GRAB1 = .96f;
    static final float SERVO_FOUNDATION_REST1 = .33f;
    static final float SERVO_FOUNDATION_GRAB2 = .91f;
    static final float SERVO_FOUNDATION_REST2 = .65f;
    static final float SERVO_AUTONOMOUS_GRABBER_GRAB = .75f;
    static final float SERVO_AUTONOMOUS_GRABBER_SCORE = 0f;
    static final float SERVO_AUTONOMOUS_UP_ARM = .32f;
    static final float SERVO_AUTONOMOUS_DOWN_ARM = .77f;
    static final float SERVO_TELEOP_ARM_POSITION = .15f;

    //********** Touch Sensor Indexes **********//
    static final int TOUCH_ARM_BOTTOM = 0;
    static final int TOUCH_ARM_TOP = 1;

    //********** CalcMove Constants **********//
    static final float WHEEL_DIAMETER = 9f;
    static final float WHEEL_GEAR_RATIO = (1f / 1);
    static final float NEVEREST_40_REVOLUTION_ENCODER_COUNT = 1120f;
    static final float TRACK_DISTANCE = 30f;

    public enum Direction {FORWARD, BACKWARD, LEFT, RIGHT}

    //********** TensorFlow **********//

    static final String VUFORIA_KEY = "ARSzhHP/////AAABmQ3dyIKKfkcipjZh0HtnoDEkjuCn18CTNUWRN7PTFoedxZLS+QZmpkyXpQnQXFpQ5ol//l0ZwTejVrGRQ4i/kQBrrFJ8E0C7ckr4lzf5bLCvi1/E9x8anPwt2D0UToZ3MB5jPx4T6s/EOs575BtxjL7uv5jrCbQDsXebm2PROU4zC/Dj7+AYFkKCqD3YYLbGPGV4YoSgp9Ihoe+ZF/eae0FLG8K/o4eyfZj0B3aXkRvYi3dC5LY+c76aU72bKTrQ2PDYSxDG8xCaY1JyEyfDA6XqjHjYMvh0BBbb8bAQvPgG6/G50+5L+c/a8u6sbYJLbvVtXdMtrG1EA4CglbnsDs7GyyJmH5AusSwIDb9DQnTA";

    static final Object Coordinates = new Object();

    public static class Coordinates {
        public double xPosition;
        public double yPosition;

        public Coordinates(double xPosition, double yPosition) {
            this.xPosition = xPosition;
            this.yPosition = yPosition;
        }
    }
}