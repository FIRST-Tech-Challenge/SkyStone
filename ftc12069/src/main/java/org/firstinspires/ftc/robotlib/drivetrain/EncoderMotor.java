package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This motor improves the base motor by using encoders to determine state
 * The base motor has constant issues with moving to a target position
 */
public class EncoderMotor extends ModifiedMotor {
    // Timer
    private ElapsedTime elapsedTime;

    // Storing values for velocity calculation
    private double lastEncoderCheck;
    private double newEncoderCheck;

    /**
     * Creates an encoder motor based off the base FTC DcMotor
     * @param motor FTC DcMotor
     */
    public EncoderMotor(DcMotor motor) {
        super(motor);

        // Init timer
        this.elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.elapsedTime.startTime();

        // Init encoder variables
        lastEncoderCheck = 0;
        newEncoderCheck = motor.getCurrentPosition();
    }

    /**
     * Calculates the encoder ticks per second using the last recorded data
     * @return Double representing an approximation encoder ticks per second
     */
    public double getEncoderTicksPerSecond() {
        // Update state variables to track elapsed time and change encoderTicks
        lastEncoderCheck = newEncoderCheck;
        newEncoderCheck = (double) this.motor.getCurrentPosition();
        double encoderTime = elapsedTime.seconds();

        // Reset the timer for next check and return change in encoderTicks over change in time
        elapsedTime.reset();
        return (newEncoderCheck - lastEncoderCheck)/encoderTime;
    }

    /**
     * Checks if the motor encoder is still detecting movement
     */
    public boolean isEncoderBusy() {
        return getEncoderTicksPerSecond() != 0;
    }
}
