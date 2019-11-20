package org.firstinspires.ftc.robotlib.controller;

import org.firstinspires.ftc.robotlib.sensor.DerivativeSensor;
import org.firstinspires.ftc.robotlib.sensor.Sensor;

public class IntegratedController implements Controller, Targetable
{
    public Sensor sensor;
    public ControlAlgorithm algorithm;

    private double target;

    public IntegratedController(Sensor sensor, ControlAlgorithm algorithm)
    {
        this.sensor = sensor;
        this.algorithm = algorithm;
    }

    public void setTarget(double target)
    {
        this.target = target;
        algorithm.setTarget(target);
        update();
    }

    public double getTarget()
    {
        return this.target;
    }

    public double update()
    {
        if (sensor instanceof DerivativeSensor && algorithm instanceof DerivativeAlgorithm)
        {
            //if both sensor and algorithm can handle derivatives pass on derivative
            ((DerivativeAlgorithm) algorithm).setDerivative(((DerivativeSensor) sensor).getDerivative()); //essentially algorithm.setDerivative(sensor.getDerivative)
        }
        double sensorValue = sensor.getValue();
        algorithm.input(sensorValue);
        return sensorValue;
    }

    @Override
    public double output()
    {
        return algorithm.output();
    }

    public double getSensorValue()
    {
        return sensor.getValue();
    }
}
