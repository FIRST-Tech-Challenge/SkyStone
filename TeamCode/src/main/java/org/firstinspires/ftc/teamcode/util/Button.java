package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Button {

    private boolean isPressed;
    private boolean shouldExecuteAction;
    private Gamepad gamepad;
    private ListenButton currentButton;

    public enum ListenButton{
        A, B, RIGHT_BUMPER, LEFT_BUMPER, X, Y
    }

    public Button(Gamepad gamepad, ListenButton currentButton){
        isPressed = false;
        shouldExecuteAction = false;
        this.gamepad = gamepad;
        this.currentButton = currentButton;
    }

    public void update(){
        boolean ifPressed = false;
        switch(currentButton){
            case A:
                ifPressed = gamepad.a;
                break;
            case B:
                ifPressed = gamepad.b;
                break;
            case RIGHT_BUMPER:
                ifPressed = gamepad.right_bumper;
                break;
            case LEFT_BUMPER:
                ifPressed = gamepad.left_bumper;
                break;
            case X:
                ifPressed = gamepad.x;
                break;
            case Y:
                ifPressed = gamepad.y;
                break;
        }
        if(!isPressed && ifPressed){
            isPressed = true;
            shouldExecuteAction = true;
        } else if(isPressed && ifPressed){
            shouldExecuteAction = false;
        }
        if(!ifPressed){
            isPressed = false;
        }
    }

    public boolean shouldExecuteAction(){
        return shouldExecuteAction;
    }
}
