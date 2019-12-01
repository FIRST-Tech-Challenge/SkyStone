package org.firstinspires.ftc.robotlib.state;

/**
 * Stores the state of Servo(s) for ease of use
 */
public enum ServoState {
    UNKNOWN(-1, -1),
    STOWED(0, 0),
    DOWN(1, 1);

    private int level;
    private int position;

    ServoState(int level, int position) {
        this.level = level;
        this.position = position;
    }

    public int getLevel() {
        return level;
    }

    public int getPosition() {
        return position;
    }

    // This auto corrects if the value is too high or low
    public static ServoState getServoStateFromInt(int level) {
        if (level > values().length - 1) return getServoStateFromInt(values().length - 1);
        if (level < 0) return getServoStateFromInt(0);

        for (ServoState servoState : values()) {
            if (servoState.getLevel() == level) return servoState;
        }

        return UNKNOWN; // this shouldn't happen
    }
}
