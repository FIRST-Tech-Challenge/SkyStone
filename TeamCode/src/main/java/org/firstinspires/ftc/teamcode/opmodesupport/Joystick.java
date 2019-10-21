package org.firstinspires.ftc.teamcode.opmodesupport;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

/**
 * Created by FIXIT on 16-10-25.
 */
public class Joystick implements Cloneable {

    private long[] lastTimeCalled = new long[14];
    private final static int TAP_REFRESH_PERIOD = 500;

    public Gamepad gamepad;

    public Joystick() {
        Arrays.fill(lastTimeCalled, 0);
    }//Joystick

    public void update(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean buttonA() {
        return gamepad.a;
    }

    public boolean buttonB() {
        return gamepad.b;
    }

    public boolean buttonX() {
        return gamepad.x;
    }

    public boolean buttonY() {
        return gamepad.y;
    }


    public boolean buttonUp() {
        return gamepad.dpad_up;
    }

    public boolean buttonDown() {
        return gamepad.dpad_down;
    }

    public boolean buttonRight() {
        return gamepad.dpad_right;
    }

    public boolean buttonLeft() {
        return gamepad.dpad_left;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean leftTrigger(){
        return (gamepad.left_trigger > 0.1);
    }

    public boolean rightTrigger(){
        return (gamepad.right_trigger > 0.1);
    }

    public boolean buttonStart() {
        return gamepad.start;
    }

    public boolean buttonBack() {
        return gamepad.back;
    }


    public float x1(){
        return (Math.abs(gamepad.left_stick_x) > 0.09) ? gamepad.left_stick_x : 0;
    }

    public float x2(){
        return (Math.abs(gamepad.right_stick_x) > 0.09)? gamepad.right_stick_x : 0;
    }

    public float y1(){
        return (Math.abs(gamepad.left_stick_y) > 0.09)? gamepad.left_stick_y : 0;
    }

    public float y2(){
        return (Math.abs(gamepad.right_stick_y) > 0.09)? gamepad.right_stick_y : 0;
    }

    public Joystick clone() {
        Joystick clone = new Joystick();
        clone.update(gamepad);
        return clone;
    }

}
