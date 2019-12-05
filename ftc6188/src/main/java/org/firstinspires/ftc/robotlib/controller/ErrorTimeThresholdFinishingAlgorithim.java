package org.firstinspires.ftc.robotlib.controller;

public class ErrorTimeThresholdFinishingAlgorithim extends FinishingAlgorithm
{
    public double errorTolerance;

    private long timeThreshold;
    private long lastOutOfRange;

    private boolean withinRange = false;

    private double input;

    public ErrorTimeThresholdFinishingAlgorithim(double errorTolerance, double timeThreshold)
    {
        this.errorTolerance = errorTolerance;
        this.timeThreshold = (long)(timeThreshold*1E9); //convert to nanosec
    }

    public ErrorTimeThresholdFinishingAlgorithim(ErrorTimeThresholdFinishingAlgorithim original, boolean copyState)
    {
        this(original.getErrorTolerance(), original.getTimeThreshold());

        if (copyState)
        {
            input = original.getInput();
            lastOutOfRange = original.getLastOutOfRange();
            withinRange = original.isWithinRange();
        }
    }

    public ErrorTimeThresholdFinishingAlgorithim(ErrorTimeThresholdFinishingAlgorithim original)
    {
        this(original, false);
    }

    @Override
    public boolean finished()
    {
        return ((System.nanoTime()-lastOutOfRange) > timeThreshold) && withinRange;
    }

    @Override
    public void input(double input)
    {
        this.input = input;

        if(Math.abs(getTarget()-input) > errorTolerance)
        {
            lastOutOfRange = System.nanoTime();
            withinRange = false;
        }
        else
        {
            withinRange = true;
        }
    }

    public double getErrorTolerance()
    {
        return errorTolerance;
    }

    public void setErrorTolerance(double errorTolerance)
    {
        this.errorTolerance = errorTolerance;
    }

    public long getTimeThreshold()
    {
        return timeThreshold;
    }

    public void setTimeThreshold(long timeThreshold)
    {
        this.timeThreshold = timeThreshold;
    }

    public long getLastOutOfRange()
    {
        return lastOutOfRange;
    }

    public  boolean isWithinRange()
    {
        return withinRange;
    }

    public double getInput()
    {
        return input;
    }
}
