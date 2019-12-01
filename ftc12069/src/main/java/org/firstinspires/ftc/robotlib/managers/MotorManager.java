package org.firstinspires.ftc.robotlib.managers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

public class MotorManager {
    private DcMotor[] motors;
    private double velocity;

    /**
     * Creates a manager for a group of motors
     * @param motors Array of DcMotors
     */
    public MotorManager(DcMotor[] motors) {
        this.motors = motors;
    }

    public double getVelocity() {
        return velocity;
    }

    /**
     * Sets the velocity of the motors
     * @param velocity new velocity
     */
    public void setMotorsVelocity(double velocity) {
        for (DcMotor motor : motors) {
           if (motor.getPower() != velocity) motor.setPower(velocity);
        }
        this.velocity = velocity;
    }

    /**
     * Stops the motors
     */
    public void stop() {
        this.setMotorsVelocity(0);
    }
}
