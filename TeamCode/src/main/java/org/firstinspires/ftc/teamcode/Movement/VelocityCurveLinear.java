package org.firstinspires.ftc.teamcode.Movement;

public class VelocityCurveLinear {

    private double maxVel, minVelAccel, minVelDeccel;
    private double accelRange;
    private double deccelRange;
    private double distance;

    private double aSlope;
    private double dSlope;

    public VelocityCurveLinear(double maxVel, double minVelAccel, double minVelDeccel, double accelRange, double deccelRange, double distance){

        this.maxVel = maxVel;
        this.minVelAccel = minVelAccel;
        this.minVelDeccel = minVelDeccel;
        this.accelRange = accelRange;
        this.deccelRange = deccelRange;
        this.distance = distance;
        //                                                                ___
        aSlope = (maxVel-minVelAccel)/(accelRange-0); //Rise over run ___/                ___
        dSlope = (maxVel-minVelDeccel)/((distance-deccelRange)-distance); //Rise over run    \___
        // Verified ^^

    }

    public double getOutput(double currentDistance){

        double output = 0;
        double input = 0;

        if(distance > accelRange){ // If there is enough distance to fully accelerate
            input = distance - currentDistance;

            if(input < accelRange && input >= 0){ // If inside the acceleration range
                output = aSlope * input + minVelAccel;
            }else if(input > accelRange && input < (distance-deccelRange)){ // If inside the maxVel range
                output = maxVel;
            }else if(input > (distance-deccelRange) && input < distance){ // If inside the de-accelerate range
                output = dSlope * (input-distance+deccelRange) + minVelDeccel;
            }

        }else{
            output = (2*minVelAccel+3*maxVel)/5; // Output 60% power
        }

        return output;

    }
}