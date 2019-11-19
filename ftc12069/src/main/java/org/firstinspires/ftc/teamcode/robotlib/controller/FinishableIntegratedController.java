package org.firstinspires.ftc.teamcode.robotlib.controller;


import org.firstinspires.ftc.teamcode.robotlib.sensor.Sensor;

public class FinishableIntegratedController extends IntegratedController
{
    FinishingAlgorithm finisher;

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
