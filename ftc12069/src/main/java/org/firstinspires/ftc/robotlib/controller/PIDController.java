package org.firstinspires.ftc.robotlib.controller;

import com.qualcomm.robotcore.util.Range;

public class PIDController extends ControlAlgorithm implements DerivativeAlgorithm
{
    private double KP;
    private double KI;
    private double KD;
    private double error = 0;
    private double target = 0;
    private double integral = 0;
    private double derivative = 0;

    private long timeAtUpdate = 0;

    private boolean integralSet = false;
    private boolean derivativeSet = false;

    private double derivativeAveraging = 0.95;

    private boolean processManualDerivative = false;

    private double maxErrorForIntegral = Double.POSITIVE_INFINITY;
    private double maxIntegral = Double.POSITIVE_INFINITY;
    private double maxDerivative = Double.POSITIVE_INFINITY;

    public PIDController(final double KP, final double KI, final double KD)
    {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        timeAtUpdate = System.nanoTime();
        integral = 0;
    }

    public PIDController(PIDController original, boolean copyState)
    {
        this(original.getKP(), original.getKI(), original.getKD());

        setDerivativeAveraging(original.getDerivativeAveraging());
        setMaxDerivative(original.getMaxDerivative());
        setMaxErrorForIntegral(original.getMaxErrorForIntegral());
        setMaxIntegral(original.getMaxIntegral());

        if (copyState)
        {
            error = original.getError();
            setTarget(original.getTarget());
            setIntegral(original.getIntegral());
            setDerivative(original.getDerivative());
        }
    }

    public PIDController(PIDController original)
    {
        this(original, false);
    }

    public void resetIntegration()
    {
        integral = 0;
    }

    private double nanoToUnit(long nano)
    {
        return nano/1E9;
    }

    @Override
    public void setTarget(double target)
    {
        this.target = target;
    }

    @Override
    public double getTarget()
    {
        return target;
    }

    @Override
    public double output()
    {
        return KP*error+KI*integral+KD*derivative;
    }

    @Override
    public void input(double input)
    {
        long newTime = System.nanoTime();
        error = target-input;
        if (!integralSet)
        {
            integral += Range.clip(error, -maxErrorForIntegral, maxErrorForIntegral)*nanoToUnit(newTime-timeAtUpdate);
            integral = Range.clip(integral, -maxIntegral, maxIntegral);
        }
        if (!derivativeSet)
        {
            derivative = derivative*derivativeAveraging+(error/nanoToUnit(newTime-timeAtUpdate))*(1-derivativeAveraging);
            derivative = Range.clip(derivative, -maxDerivative, maxDerivative);
        }
        timeAtUpdate = newTime;
        integralSet = false;
        derivativeSet = false;
    }

    public double getIntegral()
    {
        return integral;
    }

    public void setIntegral(double integral)
    {
        this.integral = integral;
        integralSet = true;
    }

    public double getDerivative()
    {
        return derivative;
    }

    public void setDerivative(double derivative)
    {
        if (processManualDerivative)
        {
            this.derivative = this.derivative*derivativeAveraging+derivative*(1-derivativeAveraging);
            this.derivative = Range.clip(this.derivative, -maxDerivative, maxDerivative);
        }
        else
        {
            this.derivative = derivative;
        }
        derivativeSet = true;
    }

    public double getKP()
    {
        return KP;
    }

    public void setKP(double KP)
    {
        this.KP = KP;
    }

    public double getKI()
    {
        return KI;
    }

    public void setKI(double KI)
    {
        this.KI = KI;
    }

    public double getKD()
    {
        return KD;
    }

    public void setKD(double KD)
    {
        this.KD = KD;
    }

    public double getError()
    {
        return error;
    }

    public double getDerivativeAveraging()
    {
        return derivativeAveraging;
    }

    public void setDerivativeAveraging(double derivativeAveraging)
    {
        this.derivativeAveraging = derivativeAveraging;
    }

    public double getMaxErrorForIntegral()
    {
        return maxErrorForIntegral;
    }

    public void setMaxErrorForIntegral(double maxErrorForIntegral)
    {
        this.maxErrorForIntegral = Math.abs(maxErrorForIntegral);
    }

    public boolean isProcessManualDerivative()
    {
        return processManualDerivative;
    }

    public void setProcessManualDerivative(boolean processManualDerivative)
    {
        this.processManualDerivative = processManualDerivative;
    }

    public double getMaxDerivative()
    {
        return maxDerivative;
    }

    public void setMaxDerivative(double maxDerivative)
    {
        this.maxDerivative = Math.abs(maxDerivative);
    }

    public double getMaxIntegral()
    {
        return maxIntegral;
    }

    public void setMaxIntegral(double maxIntegral)
    {
        this.maxIntegral = Math.abs(maxIntegral);
    }


}
