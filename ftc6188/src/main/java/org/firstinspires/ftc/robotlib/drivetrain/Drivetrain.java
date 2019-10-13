package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

abstract public class Drivetrain
{
    private double velocity = 0;

    private ToggleBoolean halfPower;

    public DcMotor[] motorList;
    private double[] motorPowers;

    public Drivetrain(DcMotor[] motorList)
    {
        this.motorList = motorList;
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
        for (int motorIndex = 0; motorIndex < motorPowers.length; motorIndex++)
        {
            motorList[motorIndex].setPower(motorPowers[motorIndex] * (halfPower.output() ? 0.5 : 1));
        }
    }

    public void halfPowerInput(boolean currentlyPressed)
    {
        halfPower.input(currentlyPressed);
    }

    abstract protected double[] calculateMotorPowers();
}
