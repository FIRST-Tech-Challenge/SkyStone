package org.firstinspires.ftc.robotlib.state;

/**
 * Stores the state of Servo(s) for ease of use
 */
public enum ServoState {
    UNKNOWN(-1),
    STOWED(0),
    DOWN(1);

    private int level;

    ServoState(int level) {
        this.level = level;
    }

    public int getLevel() {
        return level;
    }

    // This auto corrects if the value is too high or low
    public static ServoState getServoStateFromInt(int level) {
        if (level > values().length - 1) return getServoStateFromInt(values().length - 1);
        if (level < 0) return getServoStateFromInt(0);

        for (ServoState servoState : values()) {
            if (servoState.getLevel() == level) return servoState;
        }

        return STOWED; // this shouldn't ever happen
    }
}
