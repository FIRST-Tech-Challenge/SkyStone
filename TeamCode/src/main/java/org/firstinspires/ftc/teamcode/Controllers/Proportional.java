package org.firstinspires.ftc.teamcode.Controllers;

/*
This is a simple proportional controller.
*/

public class Proportional extends Controller {

    private double pGain;
    private double limit;

    public Proportional(double p_Gain, double limit) {

        this.pGain = p_Gain;
        this.limit = limit;

    }

    public void update(double target, double current) {

        error = target - current;

        correction = error * pGain;

        if(correction > limit) {
            correction = limit;
        }else if(correction < -limit) {
            correction = -limit;
        }

    }

}