package org.firstinspires.ftc.robotlib.controller;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller, which allows the robot to move to a specific position and correct itself so that it lands right on (or very close).
 * It can also keep a robot on course if it is moving forward and is bumped.
 * The P stands for proportional, the I stands for integral, and the D stands for derivative.
 */
public class PIDController extends ControlAlgorithm implements DerivativeAlgorithm {
    /**
     * The gain for the proportional part of the controller.
     */
    private double KP;

    /**
     * The gain for the integral part of the controller.
     */
    private double KI;

    /**
     * The gain for the derivative part of the controller.
     */
    private double KD;

    /**
     * The output from the proportional part of the controller.
     */
    private double error = 0;

    /**
     * The position the robot is trying to get to.
     */
    private double target = 0;

    /**
     * The ouptut from the integral part of the controller.
     */
    private double integral = 0;

    /**
     * The output from the derivative part of the controller.
     */
    private double derivative = 0;

    /**
     * The time, in nanoseconds, of the last time of controller ran through the loop. Used when calculating the integral and derivative.
     */
    private long timeAtUpdate = 0;

    private boolean integralSet = false;
    private boolean derivativeSet = false;

    private double derivativeAveraging = 0.95;

    private boolean processManualDerivative = false;

    private double maxErrorForIntegral = Double.POSITIVE_INFINITY;
    private double maxIntegral = Double.POSITIVE_INFINITY;
    private double maxDerivative = Double.POSITIVE_INFINITY;

    /**
     * The constructor for creating the PID Controller.
     * @param KP The gain for the derivative part of the controller
     * @param KI The gain for the integral part of the controller
     * @param KD The gain for the derivative part of the controller
     */
    public PIDController(final double KP, final double KI, final double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        timeAtUpdate = System.nanoTime();
        integral = 0;
    }

    public PIDController(PIDController original, boolean copyState) {
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

    public PIDController(PIDController original) {
        this(original, false);
    }

    /**
     * Sets the value of the integral to zero again.
     */
    public void resetIntegration() {
        integral = 0;
    }

    /**
     * Converts nanoseconds to seconds.
     * @param nano The time in nanoseconds
     * @return The time in seconds
     */
    private double nanoToUnit(long nano) {
        return nano/1E9;
    }

    /**
     * Receives a target value from the user.
     * @param target The value the system is trying to reach or maintain
     */
    @Override
    public void setTarget(double target) {
        this.target = target;
    }

    /**
     * Gets the target value the system is currently trying to reach or maintain.
     * @return The current target value
     */
    @Override
    public double getTarget() {
        return target;
    }

    /**
     * After doing a lot of math, the controller uses the inputs to calculate a value which the system is able to understand
     * and use to change an attribute such as velocity or course.
     * @return The calculations of all of the different parts of the controller multiplied by the respective gains
     */
    @Override
    public double output() {
        return KP*error+KI*integral+KD*derivative;
    }

    /**
     * Does all of the fancy math, requiring only an input into the proportional part of the controller.
     * @param input The proportional input into the fancy math, such as the heading from a gyro
     */
    @Override
    public void input(double input) {
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

    /**
     * Gets the value of the integral part of the controller.
     * @return The output of the integral calculation
     */
    public double getIntegral() {
        return integral;
    }

    public void setIntegral(double integral) {
        this.integral = integral;
        integralSet = true;
    }

    /**
     * Gets the value of the derivative part of the controller.
     * @return The output of the derivative calculation
     */
    public double getDerivative() {
        return derivative;
    }

    public void setDerivative(double derivative) {
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

    public double getKP() {
        return KP;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    public double getKD() {
        return KD;
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getError() {
        return error;
    }

    public double getDerivativeAveraging() {
        return derivativeAveraging;
    }

    public void setDerivativeAveraging(double derivativeAveraging)
    {
        this.derivativeAveraging = derivativeAveraging;
    }

    public double getMaxErrorForIntegral() {
        return maxErrorForIntegral;
    }

    public void setMaxErrorForIntegral(double maxErrorForIntegral)
    {
        this.maxErrorForIntegral = Math.abs(maxErrorForIntegral);
    }

    public boolean isProcessManualDerivative() {
        return processManualDerivative;
    }

    public void setProcessManualDerivative(boolean processManualDerivative)
    {
        this.processManualDerivative = processManualDerivative;
    }

    public double getMaxDerivative() {
        return maxDerivative;
    }

    public void setMaxDerivative(double maxDerivative)
    {
        this.maxDerivative = Math.abs(maxDerivative);
    }

    public double getMaxIntegral() {
        return maxIntegral;
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }


}
