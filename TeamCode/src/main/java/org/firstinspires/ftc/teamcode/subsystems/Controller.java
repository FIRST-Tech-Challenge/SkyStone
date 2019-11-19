package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller extends Gamepad {
    //This is the gamepad passed in from init in a teleop mode
    Gamepad controller;
    //These are modifiers to the valaues the gamepad gives us
    double leftStickXMod = 1;
    double leftStickYMod = -1;
    double rightStickXMod = 1;
    double rightStickYMod = -1;
    boolean aMod = true;
    boolean bMod = true;
    boolean xMod = true;
    boolean yMod = true;
    boolean rightBumperMod = true;
    boolean leftBumperMod = true;
    double rightTriggerMod = 1.0;
    double leftTriggerMod = 1.0;

    //Records last button press to deal with single button presses doing a certain methods
    boolean aLast = false;
    boolean bLast = false;
    boolean xLast = false;
    boolean yLast = false;
    boolean rightBumperLast = false;
    boolean leftBumperLast = false;

    //Use given controller in teleop mode
    public Controller(Gamepad controller) {
        this.controller = controller;
    }

    //these return the value given times the modifier
    public double getLeftStickX() {
        return controller.left_stick_x * leftStickXMod;
    }

    public double getLeftStickY() {
        return controller.left_stick_y * leftStickYMod;
    }

    public double getRightStickX() {
        return controller.right_stick_x * rightStickXMod;
    }

    public double getRightStickY() {
        return controller.right_stick_y * rightStickYMod;
    }

    public double getRightTrigger() {
        return controller.right_trigger * rightTriggerMod;
    }

    public double getLeftTrigger() {
        return controller.left_trigger * leftTriggerMod;
    }

    //These methods return the oppisite of the button if the modifier is false and the same if it is true
    public boolean getA() {
        return aMod ? controller.a : !controller.a;
    }

    public boolean getB() {
        return bMod ? controller.b : !controller.b;
    }

    public boolean getX() {
        return xMod ? controller.x : !controller.x;
    }

    public boolean getY() {
        return yMod ? controller.y : !controller.y;
    }

    public boolean getLeftBumper() {
        return aMod ? controller.left_bumper : !controller.left_bumper;
    }

    public boolean getRightBumper() {
        return aMod ? controller.right_bumper : !controller.right_bumper;
    }

    public boolean getLeftBumperPress() {
        boolean isPress = getLeftBumper() ^ leftBumperLast;
        leftBumperLast = getLeftBumper();
        return isPress;
    }

    public boolean getRightBumperPress() {
        boolean isPress = getRightBumper() ^ rightBumperLast;
        rightBumperLast = getRightBumper();
        return isPress;
    }

    public boolean getAPress() {
        boolean isPress = getA() ^ aLast;
        aLast = getA();
        return isPress;
    }

    public boolean getBPress() {
        boolean isPress = getB() ^ bLast;
        leftBumperLast = getB();
        return isPress;
    }

    public boolean getXPress() {
        boolean isPress = getX() ^ xLast;
        leftBumperLast = getX();
        return isPress;
    }

    public boolean getYPress() {
        boolean isPress = getY() ^ yLast;
        leftBumperLast = getY();
        return isPress;
    }

    //method is used to convert linear map from contorller input to power into a cubic map
    public double limitStick(double stickInput) {
        return stickInput * stickInput * stickInput;
    }

    public double sensitiveStick(double stickInput) {
        if (getRightTrigger() > .5) {
            return limitStick(stickInput) * .25;
        } else {
            return limitStick(stickInput);
        }
    }
}
