package org.firstinspires.ftc.robotlib.managers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

/**
 * Manager for treating a group of Servos as one.
 */
public class ServoManager {
    // State of managed servos
    private ServoState servoState = ServoState.STOWED;
    private Servo[] servos;

    /**
     * Creates a manager for a group of servos.
     * @param servos Array of Servos
     */
    public ServoManager(Servo[] servos) {
        this.servos = servos;
    }

    /**
     * Sets the position of the servos.
     * @param position new position
     */
    public void setPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

    /**
     * Updates the servos based on their state.
     */
    public void update() {
        this.setPosition(servoState.getPosition());
    }

    /**
     * Resets the servos to Up (1.0).
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

    /**
     * Sets the servo state and updates the servos position.
     * @param servoState New state
     */
    public void setServoState(ServoState servoState) {
        this.servoState = servoState;
        this.update();
    }
}
