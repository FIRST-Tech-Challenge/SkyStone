package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

public class CompelledServo extends ModifiedServo {
    private ServoState[] states;
    private int stateIndex = 0;
    private double offset;

    public CompelledServo(Servo servo, ServoState[] states) {
        super(servo);
        this.states = states;
    }

    public void increment() {
        this.setState(stateIndex + 1);
    }

    public void decrement() {
        this.setState(stateIndex - 1);
    }

    public void setState(ServoState newState) {
        for (int i = 0; i < states.length; i++) {
            if (states[i] == newState) {
                this.stateIndex = i;
                this.setPosition(newState.getPosition());
            }
        }
    }

    public void setState(int newIndex) {
        if (newIndex < 0 || newIndex >= states.length) return;
        this.stateIndex = newIndex;
        this.setPosition(states[newIndex].getPosition());
    }

    @Override
    public void setPosition(double position) {
        this.servo.setPosition(position + offset);
    }

    public ServoState getState() {
        return states[stateIndex];
    }

    public ServoState[] getStates() {
        return states;
    }

    public int getStateIndex() {
        return stateIndex;
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void reset() {
        this.setState(0);
    }
}
