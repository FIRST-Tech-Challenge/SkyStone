package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller extends Gamepad {
    Gamepad controller;
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

    public Controller(Gamepad controller) {
        this.controller = controller;
    }

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
        return aMod ? controller.a : !controller.a;
    }

    public boolean getRightBumper() {
        return aMod ? controller.a : !controller.a;
    }

}
