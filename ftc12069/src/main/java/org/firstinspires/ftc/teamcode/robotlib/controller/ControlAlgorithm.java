package org.firstinspires.ftc.teamcode.robotlib.controller;

public abstract class ControlAlgorithm implements Controller, TargetableAlgorithm
{
    private double target;
    public void setTarget(double target)
    {
        this.target = target;
    }

    public double getTarget()
    {
        return this.target;
    }
}
