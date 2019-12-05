package org.firstinspires.ftc.robotlib.controller;

import org.firstinspires.ftc.robotlib.sensor.Sensor;

public class FinishableIntegratedController extends IntegratedController
{
    private FinishingAlgorithm finisher;

    public FinishableIntegratedController(Sensor sensor, ControlAlgorithm algorithm, FinishingAlgorithm finisher)
    {
        super(sensor, algorithm);
        this.finisher = finisher;
    }

    public void setTarget(double target)
    {
        finisher.setTarget(target);
        super.setTarget(target);
    }

    @Override
    public double update()
    {
        double sensorValue = super.update();
        finisher.input(sensorValue);
        return sensorValue;
    }

    public boolean finished()
    {
        return finisher.finished();
    }
}
