package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.glob.SharedTelemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ManyMotors {
    // list of motors
    public final List<DcMotor> motors;

    /**
     * @param motors        list of motors
     */
    public ManyMotors(List<DcMotor> motors) {
        this.motors = motors;
    }

    /**
     * run the motors with raw power/speed
     * @param values the raw values to send to the motors
     */
    public void move(List<Double> values) {
        //set the motor powers/speeds of each motor]

        for (int i = 0; i < motors.size(); i++) {
            Double value = values.get(i);
            DcMotor motor = motors.get(i);
            motor.setPower(value);
        }
    }

    /**
     * stop all the motors
     */
    public void stop() {
        //create a list of the same length as motors filled with zeroes
        move(new ArrayList<>(Collections.nCopies(motors.size(), 0.0)));
    }
}