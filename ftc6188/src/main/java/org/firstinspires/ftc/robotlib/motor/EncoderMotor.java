package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
The encoder motor uses an encoder to determine the actual rotational velocity of the motor in encoder ticks per second and rotations per second
The encoder motor is used in autonomous to provide a more accurate stopping point in the code as a regular motor often has problems reporting
when it is in its "finished" state
 */
public class EncoderMotor extends ModifiedMotor
{
    // Timer
    private ElapsedTime elapsedTime;

    // Storing values for velocity calculation
    private double lastEncoderCheck;
    private double newEncoderCheck;

    public EncoderMotor(DcMotor motor)
    {
        super(motor);

        // Init timer
        this.elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.elapsedTime.startTime();

        // Init state variables
        lastEncoderCheck = 0;
        newEncoderCheck = motor.getCurrentPosition();
    }

    public double getRevolutionsPerSecond() { return getEncoderTicksPerSecond()/this.getMotorType().getTicksPerRev(); }

    public double getEncoderTicksPerSecond()
    {
        // Update state variables to track elapsed time and change encoderTicks
        lastEncoderCheck = newEncoderCheck;
        newEncoderCheck = (double)this.motor.getCurrentPosition();
        double encoderTime = elapsedTime.seconds();

        // Reset the timer for next check and return change in encoderTicks over change in time
        elapsedTime.reset();
        return (newEncoderCheck - lastEncoderCheck)/encoderTime;
    }

    // Returns true if the encoder is still running
    public boolean isEncoderBusy() { return getEncoderTicksPerSecond() != 0; }

    @Override
    public void setPower(double power) { this.motor.setPower(power); }
}
