package org.eastsideprep.murderbot;

/**
 * Created by gmein on 2/6/2018.
 */

import com.qualcomm.robotcore.util.ElapsedTime;


public class AverageValue {
    interface ValueGetter {
        double getValue();
    }

    private double value;
    private double newWeight;
    private double trigger;
    private ValueGetter vg;
    private ElapsedTime period = new ElapsedTime();
    private double triggerTime;
    private double triggerMin;


    AverageValue(double newWeight, int triggerPercent, double triggerMin, ValueGetter vg, double initialValue) {
        this.newWeight = newWeight;
        this.value = initialValue;
        this.trigger = triggerPercent;
        this.vg = vg;
        this.triggerMin = triggerMin;
    }

    double getValue() {
        return this.value;
    }

    // returns whether the value exceeded trigger points low or high
    int queryTrigger() {
        int result = 0;

        double v = vg.getValue();
        if (v / value > (1 + trigger / 100)) {
            if (period.milliseconds() > triggerTime + triggerMin) {
                result = 1;
                triggerTime = period.milliseconds();
            }
        } else if (v / value < (1 / (1 + trigger / 100))) {
            if (period.milliseconds() > triggerTime + triggerMin) {
                result = -1;
                triggerTime = period.milliseconds();
            }
        }
        if (period.milliseconds() > triggerTime + triggerMin) {
            this.value = this.value * (1.0 - newWeight) + v * this.newWeight;
        }
        return result;
    }
}
