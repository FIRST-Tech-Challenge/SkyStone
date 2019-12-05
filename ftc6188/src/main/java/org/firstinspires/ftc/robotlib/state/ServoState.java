package org.firstinspires.ftc.robotlib.state;

/**
 * Stores the state of Servo(s) for ease of use
 */
public enum ServoState
{
    UNKNOWN(-1), STOWED(0), UP(1), DOWN(2);

    private int stateLevel;

    ServoState(int stateLevel) { this.stateLevel = stateLevel; }

    public int getStateLevel() { return stateLevel; }

    // This auto corrects if the value is too high or low
    public static ServoState getServoStateFromInt(int level)
    {
        if ((level > values().length - 1) || (level < 0)) { return UNKNOWN; }
        for (ServoState servoState : values()) { if (servoState.getStateLevel() == level) { return servoState; }}
        return UNKNOWN;
    }
}
