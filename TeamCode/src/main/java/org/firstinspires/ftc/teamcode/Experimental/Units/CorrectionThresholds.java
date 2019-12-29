package org.firstinspires.ftc.teamcode.Experimental.Units;

public class CorrectionThresholds {
    private double turnCorrectorThreshold;    //0 - 1: Allowed Error, 1 = OFF, 0 = 100% Accuracy Required
    private double driveCorrectorThreshold;   //0 - 1: Allowed Error, 1 = OFF, 0 = 100% Accuracy Required

    public CorrectionThresholds(double driveCorrectorThreshold, double turnCorrectorThreshold){
        if(turnCorrectorThreshold > 1 || turnCorrectorThreshold < 0 || driveCorrectorThreshold > 1 ||
                driveCorrectorThreshold < 0)
            throw new IllegalArgumentException("TurnCorrectorThreshold and DriveCorrectorThreshold must be => 0 and <= 1.");

        this.driveCorrectorThreshold = driveCorrectorThreshold;
        this.turnCorrectorThreshold = turnCorrectorThreshold;
    }

    public double getTurnCorrectorThreshold() {
        return turnCorrectorThreshold;
    }

    public double getDriveCorrectorThreshold() {
        return driveCorrectorThreshold;
    }

    public double getDriveCorrectionAllowedError(){
        return driveCorrectorThreshold < 1 ? 20 * driveCorrectorThreshold : 144;
    }

    public double getRotationCorrectionAllowedError(){
        return turnCorrectorThreshold < 1 ? (Math.PI / 6) * turnCorrectorThreshold : 2 * Math.PI;
    }
}
