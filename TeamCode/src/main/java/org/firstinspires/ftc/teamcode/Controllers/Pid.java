package org.firstinspires.ftc.teamcode.Controllers;

/*
This is a pretty self-explanatory PID controller. sumLimit is the limit of the error sum, and thus constrains the I-term.
 */

public class Pid extends Controller {

    private double pGain, iGain, dGain;
    private double errorSum, sumLimit, lastError, errorSlope;
    private double P, I, D, correctUpLimit, correctLowLimit;
    private boolean firstLoop = true;

    // PID constructor: kP, kI, kD, limit of errorSum, limit of correction (all limits are positive and constrained about 0)
    public Pid(double pGain, double iGain, double dGain, double sumLimit, double correctUpLimit, double correctLowLimit) {

        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
        this.sumLimit = sumLimit;
        this.correctUpLimit = correctUpLimit;
        this.correctLowLimit = correctLowLimit;

    }

    @Override
    public void update(double setPoint, double current){

        error = setPoint - current;

        errorSum += error;
        if(errorSum > sumLimit){
            errorSum = sumLimit;
        }else if(errorSum < -sumLimit){
            errorSum = -sumLimit;
        }

        if(firstLoop){
            firstLoop = false;
            errorSlope = 0;
        }else{
            errorSlope = error - lastError;
        }
        lastError = error;

        P = error * pGain;
        I = errorSum * iGain;
        D = errorSlope * dGain;

        correction = P + I + D;

        if(correction > correctUpLimit){
            correction = correctUpLimit;
        }else if(correction < -correctUpLimit){
            correction = -correctUpLimit;
        }else if(correction > 0 && correction < correctLowLimit){
            correction = correctLowLimit;
        }else if(correction < 0 && correction > -correctLowLimit){
            correction = -correctLowLimit;
        }

    }
}
