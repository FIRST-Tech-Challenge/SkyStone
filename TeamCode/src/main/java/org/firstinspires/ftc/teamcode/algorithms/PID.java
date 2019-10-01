package org.firstinspires.ftc.teamcode.algorithms;

public class PID {

    /**
     * How heavily the proportional error is weighted
     */
    double kP;

    /**
     * How heavily the integral of error is weighted
     */
    double kI;

    /**
     * How heavily the derivative of error is weighted
     */
    double kD;

    /**
     * Stores the proportion of error
     */
    double proportional;

    /**
     * stores the integral of error
     */
    double integral;

    /**
     * stores the derivative of error
     */
    double derivative;

    /**
     * The current value of the variable being tracked
     */
    double processVariable;

    /**
     * Desired value for process variable
     */
    double setPoint;

    /**
     * Current deviation of setPoint from process variable
     */
    double currentError;

    /**
     * Deviation of error from process variable step before
     */
    double pastError;

    /**
     * Based on PID, how to change process variable
     */
    double output;

    /**
     * Default constructor
     */
    public PID() {
    }

    /**
     * Constructor for a PID that takes values for gains (coefficients)
     * @param kP                gain for proportion
     * @param kI                gain for integral
     * @param kD                gain for derivative
     */
    public PID(double kP, double kI, double kD) {
        //initializes gains variables
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Constructor for PID that takes values for gains, the desired value of the process variable,
     * and its current value
     * @param kP                gain for proportion
     * @param kI                gain for integral
     * @param kD                gain for derivative
     * @param setPoint          desired value of process variable
     * @param processVariable   current value of process variable
     */
    public PID (double kP, double kI, double kD, double setPoint, double processVariable) {
        //initializes variables
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setPoint = setPoint;
        this.processVariable = processVariable;
        this.currentError = setPoint - this.processVariable;
        this.pastError = this.currentError;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getProportional() {
        return proportional;
    }

    public void setProportional(double proportional) {
        this.proportional = proportional;
    }

    public double getIntegral() {
        return integral;
    }

    public void setIntegral(double integral) {
        this.integral = integral;
    }

    public double getDerivative() {
        return derivative;
    }

    public void setDerivative(double derivative) {
        this.derivative = derivative;
    }

    public double getProcessVariable() {
        return processVariable;
    }

    public void setProcessVariable(double processVariable) {
        this.processVariable = processVariable;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getCurrentError() {
        return currentError;
    }

    public void setCurrentError(double currentError) {
        this.currentError = currentError;
    }

    public double getOutput() {
        return output;
    }

    public void setOutput(double output) {
        this.output = output;
    }

    /**
     * Calculates the error of the process variable from its desired target
     * @return                  difference of desired vs. actual
     */
    public double calcError() {
        return setPoint - processVariable;
    }

    /**
     * Updates the value for error
     */
    public void updateError() {
        //sets past error to what is now current error
        pastError = currentError;

        //updates the current error
        currentError = calcError();
    }

    /**
     * Calculates the proportion of error
     * @return              value for proportional
     */
    public double calcProportional() {
        return currentError;
    }

    /**
     * Updates proportional
     */
    public void updateProportional() {
        proportional = calcProportional();
    }

    /**
     * Calculates the unscaled integral based on the right hand rule
     * @param dt            interval of time
     * @return              value for integral
     */
    public double calcIntegralRightHand(double dt) {
        return integral + (dt * currentError);
    }

    /**
     * Calculates the unscaled integral based on the left hand rule
     * @param dt            interval of time
     * @return              value for integral
     */
    public double calcIntegralLeftHand(double dt) {
        return integral + (dt * pastError);
    }

    /**
     * Calculates the unscaled integral based on the trapezoid rule
     * @param dt            interval of time
     * @return              value for integral
     */
    public double calcIntegralTrapezoid(double dt) {
        return integral + (dt * (pastError + currentError)/2);
    }

    /**
     * Updates integral
     * @param dt            interval of time
     */
    public void updateIntegral(double dt) {
        integral = calcIntegralTrapezoid(dt);
    }

    /**
     * Calculates the derivative of error
     * @param dt            interval of time
     * @return                  value for derivative
     */
    public double calcDerivative(double dt) {
        return (currentError - pastError)/(dt);
    }

    /**
     * Updates derivative
     * @param dt            interval of time
     */
    public void updateDerivative(double dt) {
        derivative = calcDerivative(dt);
    }

    /**
     * Updates the values for the P, I, D
     * @param dt            interval of time
     */
    public void updatePID(double dt) {
        updateError();
//        System.out.printf("%25f, %25f \n", currentError, pastError);
        updateProportional();
        updateIntegral(dt);
        updateDerivative(dt);
    }

    /**
     * Calculates output of PID by scaling and summing P, I, D
     * @return                  output based on PID
     */
    public double calcOutput() {
        return (kP * proportional) + (kI * integral) + (kD * derivative);
    }

    /**
     * Updates output
     */
    public void updateOutput() {
        output = calcOutput();
    }

    /**
     * Updates output if it's lower than a specified value
     * @param limit                 maximum value of output
     */
    public void updateOutput(double limit) {
        if (calcOutput() <= limit) {
            output = calcOutput();
        }
    }
}

