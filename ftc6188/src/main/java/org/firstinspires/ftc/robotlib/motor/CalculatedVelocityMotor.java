package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CalculatedVelocityMotor extends ModifiedMotor
{
    // Timer
    private ElapsedTime elapsedTime;

    // Storing values for velocity calculation
    private double lastEncoderCheck;
    private double encoderCheck;

    public CalculatedVelocityMotor(DcMotor motor)
    {
        super(motor);
        this.elapsedTime = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        this.elapsedTime.startTime();

        lastEncoderCheck = 0;
        encoderCheck = motor.getCurrentPosition();
    }

    public double getEncoderPerSecond()
    {
        lastEncoderCheck = encoderCheck;
        encoderCheck = (double)motor.getCurrentPosition();
        long encoderTime = elapsedTime.nanoseconds();

        return (encoderCheck - lastEncoderCheck)/encoderTime;
    }

    public boolean isEncoderBusy()
    {
        return getEncoderPerSecond() != 0;
    }
}
