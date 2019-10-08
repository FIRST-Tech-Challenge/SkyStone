package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class Drivetrain
{
    private double velocity = 0;

    public DcMotor[] motorList;
    private double[] motorPowers;

    public Drivetrain(DcMotor[] motorList) { this.motorList = motorList; }

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
            motorList[motorIndex].setPower(motorPowers[motorIndex]);
        }
    }

    abstract protected double[] calculateMotorPowers();
}
