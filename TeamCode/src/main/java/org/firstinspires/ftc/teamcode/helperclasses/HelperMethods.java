package org.firstinspires.ftc.teamcode.helperclasses;

public class HelperMethods {

    // Makes class unable to be instantiated
    private HelperMethods(){    }

    /**
     * Easy way of determining if a VALUE1 is within THRESHOLD PERCENT of a VALUE2
     * @param value1 First value
     * @param value2 Second value
     * @param thresholdPercent Percentage value for which value1/value2 needs to be within
     * @return
     */
    public static boolean inThreshhold(double value1, double value2, double thresholdPercent){
        double ratio = value1 / value2;

        boolean withinUpperThreshold = ratio >= 1 + thresholdPercent * .01;
        boolean withinLowerThreshold = ratio <= 1 - thresholdPercent * .01;

        return  withinUpperThreshold || withinLowerThreshold;
    }
}
