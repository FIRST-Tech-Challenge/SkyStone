package org.firstinspires.ftc.robotlib.state;

import org.jetbrains.annotations.NotNull;

/**
 * Stores the state of Servo(s) for ease of use
 */
public enum ServoState {
    UNKNOWN(-1, -1),
    STOWED(0, 0),
    BLOCK3(1, 0),
    BLOCK2(2, 0),
    BLOCK1(3, 0),
    DOWN(4, 1);

    private int id;
    private int position;

    ServoState(int id, int position) {
        this.id = id;
        this.position = position;
    }

    public int getId() {
        return id;
    }

    public int getPosition() {
        return position;
    }

    // This auto corrects if the value is too high or low
    public static ServoState getServoStateFromInt(int id) {
        if (id > values().length - 1) return getServoStateFromInt(values().length - 1);
        if (id < 0) return getServoStateFromInt(0);

        for (ServoState servoState : values()) {
            if (servoState.getId() == id) return servoState;
        }

        return UNKNOWN; // this shouldn't happen
    }

    @NotNull
    public String toString() {
        return this.toString() + " (" + this.getId() + ")";
    }
}
