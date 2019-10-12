package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    Gamepad controller;
    double leftStickXMod;
    double leftStickYMod = -1;
    double rightStickXMod;
    double rightStickYMod = -1;
    boolean aMod;
    boolean bMod;
    boolean xMod;
    boolean yMod;
    boolean rightBumperMod;
    boolean leftBumperMod;
    double rightTriggerMod;
    double leftTriggerMod;

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

    public double getRightTrigger() {
        return controller.right_trigger * rightTriggerMod;
    }

    public double getLeftTrigger() {
        return controller.left_trigger * leftTriggerMod;
    }
}
