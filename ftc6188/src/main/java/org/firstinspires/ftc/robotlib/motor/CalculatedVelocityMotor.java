package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CalculatedVelocityMotor extends ModifiedMotor
{
    // Timer
    private ElapsedTime elapsedTime;

    // Storing values for velocity calculation
    private double lastEncoderCheck;
    private double newEncoderCheck;

    public CalculatedVelocityMotor(DcMotor motor)
    {
        super(motor);
        this.elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.elapsedTime.startTime();

        lastEncoderCheck = 0;
        newEncoderCheck = motor.getCurrentPosition();
    }

    public double getRevolutionsPerSecond()
    {
        return getEncoderTicksPerSecond()/this.getMotorType().getTicksPerRev();
    }

    public double getEncoderTicksPerSecond()
    {
        lastEncoderCheck = newEncoderCheck;
        newEncoderCheck = (double)motor.getCurrentPosition();
        double encoderTime = elapsedTime.seconds();

        elapsedTime.reset();
        return (newEncoderCheck - lastEncoderCheck)/encoderTime;
    }

    // Returns true if the encoder is still running
    public boolean isEncoderBusy()
    {
        return getEncoderTicksPerSecond() != 0;
    }
}
