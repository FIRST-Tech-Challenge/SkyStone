package org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons;

public enum GamepadButtons {

    A(9), B(8), X(7), Y(6),
    LEFT_BUMPER(1), RIGHT_BUMPER(0),
    LEFT_TRIGGER(17), RIGHT_TRIGGER(16),
    DPAD_UP(13), DPAD_DOWN(12), DPAD_LEFT(11), DPAD_RIGHT(10),
    LEFT_STICK_BUTTON(15), RIGHT_STICK_BUTTON(14),
    GUIDE(4), START(3), BACK(2);

    final int bitmask;

    GamepadButtons(int shift) {
        bitmask = 1 << shift;
    }
}
