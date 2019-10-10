package org.firstinspires.ftc.teamcode.hardware.motors;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MotorGroup {
    // list of motors
    private final List<? extends Motor> motors;

    // maximum speed
    private final double maxRobotSpeed;

    // values most recently sent to the motors
    private final double[] values;

    /**
     * @param motors        list of motors
     * @param maxRobotSpeed maximum speed
     */
    public MotorGroup(List<? extends Motor> motors, double maxRobotSpeed) {
        this.motors = motors;
        this.maxRobotSpeed = maxRobotSpeed;
        values = new double[motors.size()];
    }

    /**
     * run the motors with raw power/speed
     * @param values the raw values to send to the motors
     */
    public void move(List<Double> values) {
        if (values.size() != motors.size()) {
            throw new IllegalArgumentException("Argument 'values' must have the same length as the number of motors.");
        }

        //set the motor powers/speeds of each motor
        for (int i = 0; i < motors.size(); i++) {
            Double value = values.get(i);
            motors.get(i).setPower(value);
            this.values[i] = value;
        }
    }

    /**
     * scales all the motor values if any one power is above the maximum of 1
     * then moves
     *
     * @param values the powers/speeds to be scaled and then moved
     */
    public void moveNormalized(List<Double> values) {

        //if the inputs are too high, scale them
        double highest = 0;

        //find the magnitude of the number with the highest magnitude
        for (double n : values) {
            if (Math.abs(n) > highest) {
                highest = Math.abs(n);
            }
        }

        if (highest < 1) { //only normalize if the values are too high
            highest = 1;
        }

        //rescale the values by the highest value
        List<Double> valuesScaled = new ArrayList<>();
        for (double power : values) {
            valuesScaled.add(power / highest);
        }

        move(valuesScaled);
    }

    /**
     * @param i the index of the motor to read the value from
     * @return the values most recently sent to the motors
     */
    public double getValue(int i) {
        return values[i];
    }

    /**
     * stop all the motors
     */
    public void stop() {
        //create a list of the same length as motors filled with zeroes
        move(new ArrayList<>(Collections.nCopies(motors.size(), 0.0)));
    }
}