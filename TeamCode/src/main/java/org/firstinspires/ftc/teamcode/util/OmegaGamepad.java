package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OmegaGamepad {

    private Gamepad gamepad;
    private int a, b, x, y = 0;
    private int dpadRight, dpadLeft, dpadDown, dpadUp = 0;
    private int rightBumper, leftBumper = 0;

    public OmegaGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update(){
        if(gamepad.a){
            ++a;
        } else {
            a = 0;
        }

        if(gamepad.b){
            ++b;
        } else {
            b = 0;
        }

        if(gamepad.x){
            ++x;
        } else {
            x = 0;
        }

        if(gamepad.y){
            ++y;
        } else {
            y = 0;
        }

        if(gamepad.dpad_down){
            ++dpadDown;
        } else {
            dpadDown = 0;
        }

        if(gamepad.dpad_left){
            ++dpadLeft;
        } else {
            dpadLeft = 0;
        }

        if(gamepad.dpad_right){
            ++dpadRight;
        } else {
            dpadRight = 0;
        }

        if(gamepad.dpad_up){
            ++dpadUp;
        } else {
            dpadUp = 0;
        }

        if(gamepad.right_bumper){
            ++rightBumper;
        } else {
            rightBumper = 0;
        }

        if(gamepad.left_bumper){
            ++leftBumper;
        } else {
            leftBumper = 0;
        }

    }

    public boolean ifA(){
        if(a > 0){
            return true;
        }
        return false;
    }

    public boolean ifB(){
        if(b > 0){
            return true;
        }
        return false;
    }

    public boolean ifX(){
        if(x > 0){
            return true;
        }
        return false;
    }

    public boolean ifY(){
        if(y > 0){
            return true;
        }
        return false;
    }

    public boolean ifOnceA(){
        if(a == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceB(){
        if(b == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceX(){
        if(x == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceY(){
        if(y == 1){
            return true;
        }
        return false;
    }

    public boolean ifDPadDown(){
        if(dpadDown > 0){
            return true;
        }
        return false;
    }

    public boolean ifDPadUp(){
        if(dpadUp > 0){
            return true;
        }
        return false;
    }

    public boolean ifDPadRight(){
        if(dpadRight > 0){
            return true;
        }
        return false;
    }

    public boolean ifDPadLeft(){
        if(dpadLeft > 0){
            return true;
        }
        return false;
    }

    public boolean ifOnceDPadDown(){
        if(dpadDown == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceDPadUp(){
        if(dpadUp == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceDPadLeft(){
        if(dpadLeft == 1){
            return true;
        }
        return false;
    }

    public boolean ifOnceDPadRight(){
        if(dpadRight == 1){
            return true;
        }
        return false;
    }

    public boolean ifRightBumper(){
        if(rightBumper > 0){
            return true;
        }
        return false;
    }

    public boolean ifOnceRightBumper(){
        if(rightBumper == 1){
            return true;
        }
        return false;
    }

    public boolean ifLeftBumper(){
        if(leftBumper > 0){
            return true;
        }
        return false;
    }

    public boolean ifOnceLeftBumper(){
        if(leftBumper == 1){
            return true;
        }
        return false;
    }

}
