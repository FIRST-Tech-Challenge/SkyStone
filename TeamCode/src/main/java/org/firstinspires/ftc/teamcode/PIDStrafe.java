package org.firstinspires.ftc.teamcode;

public class PIDStrafe  {

    private double tuningP;
    private double tuningI;
    private double tuningD;

    private double inputSignal;
    private double outputSignal;

    public PIDStrafe(double tuningP, double tuningI, double tuningD){
        this.tuningP = tuningP;
        this.tuningI = tuningI;
        this.tuningD = tuningD;
    }

}
