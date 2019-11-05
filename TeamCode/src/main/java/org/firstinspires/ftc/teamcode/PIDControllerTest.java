package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

// NOT AN OPMODE
// Defines a PIDControllerTest
public class PIDControllerTest {
    double p, i, d;
    double[] inputRange = {0, 1};
    double[] outputRange = {0, 1};
    ArrayList<Double> time = new ArrayList<Double>();
    ArrayList<Double> errors = new ArrayList<Double>();
    double t = 0;

    public PIDControllerTest(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void setInputRange(double min, double max) {
        this.inputRange[0] = min;
        this.inputRange[1] = max;
    }

    public void setOutputRange(double min, double max) {
        this.outputRange[0] = min;
        this.outputRange[1] = max;
    }

    public void enable() { // starts pid loop

    }

    private double calculateIntegral() {
        return 0;
    }


}
