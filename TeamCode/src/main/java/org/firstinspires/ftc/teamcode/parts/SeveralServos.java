package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.glob.SharedTelemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SeveralServos {
    // list of motors
    public final List<Servo> servos;

    // values most recently sent to the motors
    public final double[] values;

    /**
     * @param servos        list of motors
     */
    public SeveralServos(List<Servo> servos) {
        this.servos = servos;
        values = new double[servos.size()];
    }

    /**
     * run the motors with raw power/speed
     * @param values the raw values to send to the motors
     */
    public void setPosition(List<Double> values) {
        //set the motor powers/speeds of each motor
        for (int i = 0; i < servos.size(); i++) {
            Double value = values.get(i);
            Servo servo = servos.get(i);
            servo.setPosition(value);
            this.values[i] = value;
        }
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
        setPosition(new ArrayList<>(Collections.nCopies(servos.size(), 0.0)));
    }
}