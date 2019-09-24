package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Sarthak on 6/29/2017.
 */

public class GamepadWrapper {

    private Gamepad gamepad;

    public GamepadWrapper(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public double getDistanceFromCenterLeft(){
        return Math.sqrt(Math.pow(this.leftStickX(), 2)+Math.pow(this.leftStickY(), 2));
    }

    public double getAngleLeftStick() {
        return Math.toDegrees(Math.atan2(this.leftStickX(), this.leftStickY()));

    }

    public double getDistanceFromCenterRight(){
        return Math.sqrt(Math.pow(this.rightStickX(), 2)+Math.pow(this.rightStickY(), 2));
    }

    public double getAngleRightStick() {
        return Math.toDegrees(Math.atan2(this.rightStickX(), this.rightStickY()));

    }

    public boolean a(){
        return gamepad.a;
    }

    public boolean b(){
        return gamepad.b;
    }

    public boolean x(){
        return gamepad.x;
    }

    public boolean y(){
        return gamepad.y;
    }

    public boolean dpadUp(){
        return gamepad.dpad_up;
    }

    public boolean dpadDown(){
        return gamepad.dpad_down;
    }

    public boolean dpadLeft(){
        return gamepad.dpad_left;
    }

    public boolean dpadRight(){
        return gamepad.dpad_right;
    }

    public double leftTrigger(){
        return gamepad.left_trigger;
    }

    public double rightTrigger(){
        return gamepad.right_trigger;
    }

    public boolean leftBumper(){
        return gamepad.left_bumper;
    }

    public boolean rightBumper(){
        return gamepad.right_bumper;
    }

    public double rightStickX(){
        return gamepad.right_stick_x;
    }

    public double rightStickY(){
        return -gamepad.right_stick_y;
    }

    public double leftStickX(){
        return gamepad.left_stick_x;
    }

    public double leftStickY(){
        return -gamepad.left_stick_y;
    }
}
