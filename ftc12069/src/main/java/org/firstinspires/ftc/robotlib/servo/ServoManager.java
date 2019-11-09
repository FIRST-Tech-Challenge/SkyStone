package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

public class ServoManager {
    // State of managed servos
    private ServoState servoState = ServoState.STOWED;
    private Servo[] servos;
    private boolean leftBumperToggled = false;
    private boolean rightBumperToggled = false;

    /**
     * Creates a manager for a group of servos
     * @param servos
     */
    public ServoManager(Servo[] servos) {
        this.servos = servos;
    }

    /**
     * Sets the position of the servos
     * @param position new position
     */
    public void setPosition(double position) {
        for (Servo servo : servos) {
            if (servo.getPosition() != position) servo.setPosition(position);
        }
    }

    /**
     * Handles updates to the controller
     * @param gamepad controller
     */
    public void handleUpdate(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            if (!leftBumperToggled) {
                servoState = ServoState.getServoStateFromInt(servoState.getLevel() + 1);
                leftBumperToggled = true;
            }
        } else leftBumperToggled = false;
        if (gamepad.right_bumper) {
            if (!rightBumperToggled) {
                servoState = ServoState.getServoStateFromInt(servoState.getLevel() - 1);
                rightBumperToggled = true;
            }
        } else rightBumperToggled = false;
    }

    /**
     * Updates the servos based on their state
     */
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

    /**
     * Resets the servos to Up (1.0)
     */
    public void reset() {
        this.setPosition(1.0);
    }

    /**
     * @return Current Servo State
     */
    public ServoState getServoState() {
        return servoState;
    }
}
