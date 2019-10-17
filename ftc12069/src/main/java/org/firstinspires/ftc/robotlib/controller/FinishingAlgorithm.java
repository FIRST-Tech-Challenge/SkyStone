package org.firstinspires.ftc.robotlib.controller;

public abstract class FinishingAlgorithm implements Finishable, TargetableAlgorithm
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
