package org.baconeers.common;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Base class for components used in BaconOpModes
 */

public class BaconComponent {

    private BaconOpMode opMode;

    public BaconComponent(BaconOpMode baconOpMode) {
        opMode = baconOpMode;
    }

    protected BaconOpMode getOpMode() {
        return opMode;
    }

    protected boolean buttonPressed(Gamepad gamepad, ButtonControl buttonControl) {
        boolean buttonPressed = false;
        switch (buttonControl) {

            case DPAD_UP:
                buttonPressed = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonPressed = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonPressed = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonPressed = gamepad.dpad_right;
                break;
            case A:
                // ignore if start is also pressed to avoid triggering when initialising the
                // controllers
                if (!gamepad.start) {
                    buttonPressed = gamepad.a;
                }
                break;
            case B:
                // ignore if start is also pressed to avoid triggering when initialising the
                // controllers
                if (!gamepad.start) {
                    buttonPressed = gamepad.b;
                }
                break;
            case X:
                buttonPressed = gamepad.x;
                break;
            case Y:
                buttonPressed = gamepad.y;
                break;
            case START:
                buttonPressed = gamepad.start;
                break;
            case BACK:
                buttonPressed = gamepad.back;
                break;
            case LEFT_BUMPER:
                buttonPressed = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonPressed = gamepad.right_bumper;
                break;
            case LEFT_STICK_BUTTON:
                buttonPressed = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonPressed = gamepad.right_stick_button;
                break;
        }

        return buttonPressed;
    }


}
