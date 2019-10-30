package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

public class ServoManager {
    private ServoState servoState = ServoState.STOWED;
    private Servo[] servos;
    private boolean leftBumperToggled = false;
    private boolean rightBumpberToggled = false;

    public ServoManager(Servo[] servos) {
        this.servos = servos;
    }

    public void setPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

    public void handleUpdate(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            if (!leftBumperToggled) {
                servoState = ServoState.getServoStateFromInt(servoState.getLevel() + 1);
                leftBumperToggled = true;
            }
        } else leftBumperToggled = false;
        if (gamepad.right_bumper) {
            if (!rightBumpberToggled) {
                servoState = ServoState.getServoStateFromInt(servoState.getLevel() - 1);
                rightBumpberToggled = true;
            }
        } else rightBumpberToggled = false;
    }

    public void updateServos() {
        switch (servoState) {
            case STOWED:
                this.setPosition(1.0);
            case UP:
                this.setPosition(0.9);
            case DOWN:
                this.setPosition(0.6);
        }
    }

    public void reset() {
        this.setPosition(1.0);
    }

    public ServoState getServoState() {
        return servoState;
    }
}
