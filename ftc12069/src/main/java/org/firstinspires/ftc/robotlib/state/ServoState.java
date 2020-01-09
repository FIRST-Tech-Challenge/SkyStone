package org.firstinspires.ftc.robotlib.state;

import org.jetbrains.annotations.NotNull;

/**
 * Stores the state of Servo(s) for ease of use.
 */
public enum ServoState {
    // General States
    UNKNOWN(-1),
    STOWED(0),
    DOWN(1),

    // Delivery States
    CRADLE(0.069),
    CARRY(0.119),
    THREEBLOCKHOVER(0.61),
    THREEBLOCKDEPOSIT(0.68),
    TWOBLOCKHOVER(0.74),
    TWOBLOCKDEPOSIT(0.78),
    ONEBLOCKHOVER(0.83),
    ONEBLOCKDEPOSIT(0.87),
    FLOOR(1.0),

    // Block Grabber
    OPEN(0.0),
    CLOSED(1.0);

    private double position;

    ServoState(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }

    public static ServoState getServoStateFromPosition(double position) {
        for (ServoState servoState : values()) {
            if (servoState.getPosition() == position) return servoState;
        }

        return UNKNOWN;
    }

    @NotNull
    public String stringify() {
        return this.toString() + " (" + this.getPosition() + ")";
    }
}
