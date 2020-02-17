package org.firstinspires.ftc.teamcode.Controllers;

/*
This is a gated constant controller. If the error is over a threshold, it is one constant,
if error is under the threshold it is another constant.
*/

public class GatedConstant extends Controller{

    private double constantA;
    private double constantB;
    private double thresh;

    public GatedConstant(double above, double below, double threshold) {
        this.constantA = above;
        this.constantB = below;
        this.thresh = threshold;
    }

    @Override
    public void update(double target, double current) {

        error = target - current;

        if(Math.abs(error) > thresh) {
            correction = constantA * Math.abs(error)/error;
        }else{
            correction = constantB * Math.abs(error)/error;
        }

    }

}