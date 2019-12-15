package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

/**
 * A compelled servo is a servo that can only move to certain positions.
 * This allows for changing states without knowing what they are.
 */
public class CompelledServo extends ModifiedServo {
    private ServoState[] states;
    private int stateIndex = 0;
    private double offset;

    /**
     * Creates a CompelledServo using the FTC Servo.
     * @param servo FTC Servo
     * @param states Possible states of the servo
     */
    public CompelledServo(Servo servo, ServoState[] states) {
        super(servo);
        this.states = states;
    }

    /**
     * Changes to the next state.
     */
    public void increment() {
        this.setState(stateIndex + 1);
    }

    /**
     * Changes to the past state.
     */
    public void decrement() {
        this.setState(stateIndex - 1);
    }

    /**
     * Sets the servo state and ignores the index.
     * @param newState New servo state (must be in the possible states)
     */
    public void setState(ServoState newState) {
        for (int i = 0; i < states.length; i++) {
            if (states[i] == newState) {
                this.stateIndex = i;
                this.setPosition(newState.getPosition());
            }
        }
    }

    /**
     * Sets the servo state using an index referencing the internal possible states.
     * @param newIndex New index to grab state from
     */
    public void setState(int newIndex) {
        if (newIndex < 0 || newIndex >= states.length) return;
        this.stateIndex = newIndex;
        this.setPosition(states[newIndex].getPosition());
    }

    /**
     * Sets the position of the servo, ignoring the current state.
     * @param position New position (between 0 and 1)
     */
    @Override
    public void setPosition(double position) {
        this.servo.setPosition(position + offset);
    }

    /**
     * Retrieves the "current" state.
     */
    public ServoState getState() {
        return states[stateIndex];
    }

    /**
     * Retrieves the possible states of the servo.
     * @return Array of possible states
     */
    public ServoState[] getStates() {
        return states;
    }

    /**
     * Retrieves the index of the current state in the array of possible states.
     */
    public int getStateIndex() {
        return stateIndex;
    }

    /**
     * Retrieves the offset of the servo positioning.
     * This is mainly for controlling multiple servos.
     */
    public double getOffset() {
        return offset;
    }

    /**
     * Sets the offset of the servo for positioning.
     * @see #getOffset()
     * @param offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * Resets the servo (assuming zero is the rest position).
     */
    public void reset() {
        this.setState(0);
    }
}
