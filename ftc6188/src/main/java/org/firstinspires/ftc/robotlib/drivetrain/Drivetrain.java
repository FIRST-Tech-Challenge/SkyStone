package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

abstract public class Drivetrain
{
    private double velocity = 0;

    private ToggleBoolean halfPower;

    private boolean isAutoMode;

    public DcMotor[] motorList;
    private double[] motorPowers;

    public Drivetrain(DcMotor[] motorList, boolean isAutoMode)
    {
        this.motorList = motorList;
        this.isAutoMode = isAutoMode;
        halfPower = new ToggleBoolean(false);
    }

    public double getVelocity() { return velocity; }

    public void setVelocity(double velocity)
    {
        this.velocity = velocity;
        updateMotorPowers();
    }

    protected void updateMotorPowers()
    {
        motorPowers = calculateMotorPowers();
        if (!isAutoMode())
        {
            for (int motorIndex = 0; motorIndex < motorPowers.length; motorIndex++)
            {
                motorList[motorIndex].setPower(motorPowers[motorIndex] * (halfPower.output() ? 0.5 : 1));
            }
        }
    }

    public void halfPowerInput(boolean currentlyPressed)
    {
        halfPower.input(currentlyPressed);
    }

    abstract protected double[] calculateMotorPowers();

    public boolean isAutoMode()
    {
        return isAutoMode;
    }

    public void setAutoMode(boolean isAutoMode)
    {
        this.isAutoMode = isAutoMode;
    }
}
