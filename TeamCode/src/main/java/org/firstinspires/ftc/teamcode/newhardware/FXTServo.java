package org.firstinspires.ftc.teamcode.newhardware;

import org.firstinspires.ftc.teamcode.RC;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

/**
 * Created by Nirzvi on 15-08-21.
 */
public class FXTServo implements FXTDevice {

    private Servo s;

    protected HashMap<String, Double> savedPositions = new HashMap<String, Double>();

    public FXTServo(Servo s) {
        this.s = s;
    }
    public FXTServo(String address) {
        this((Servo) RC.h.get(address));
    }

    public void addPos (String name, double position) {
        savedPositions.put(name, position);
    }

    public void removePos(String name) {
        savedPositions.remove(name);
    }

    public void goToPos(String name) {
        setPosition(savedPositions.get(name));
    }

    private double translate (int angle) {
        return angle / 180.0;
    }

    public double getPosition() {
        synchronized (s) {
            return s.getPosition();
        }//synchronized
    }

    public void setPosition(double pos) {

        if(pos > 1) pos = 1;
        if(pos < 0) pos = 0;

        synchronized (s) {
            s.setPosition(pos);
        }//synchronized

    }//setPosition

    public void add(double increment) {
        setPosition(getPosition() + increment);
    }

    public void subtract(double decrement) {
        setPosition(getPosition() - decrement);
    }

}//FXTServo
