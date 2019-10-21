package org.firstinspires.ftc.teamcode.newhardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;

import org.firstinspires.ftc.teamcode.RC;

/**
 * Created by FIX IT on 8/19/2015
 * This class allows continuous rotation servos to be used like a motor rather than a servo.
 * To find the zero position
 */
public class FXTCRServo extends CRServoImpl implements FXTDevice, Timeable {

    private long targetTime = -1;
    private double zeroPos = 0;

    private FXTCRServo(CRServo cs) {
        super(cs.getController(), cs.getPortNumber());
    }

    public FXTCRServo(String addr) {
        this(RC.h.crservo.get(addr));
    }

    public void setZeroPosition(double newZeroPos) {
        zeroPos = newZeroPos;
    }//setZeroPosition

    public double getZeroPosition() {
        return zeroPos;
    }//getZeroPosition

    public void setPower(double power) {
        if (power < 0) {
            super.setPower(power / Math.abs(-1 - zeroPos));
        } else {
            super.setPower(power / Math.abs(1 - zeroPos));
        }//else
    }//setPower

    public void stop() {
        setPower(0);
    }

    public void setTimer(long newTime, double speed) {
        setPower(speed);
        targetTime = System.currentTimeMillis() + newTime;
    }//setTimer

    @Override
    public boolean isTimeFin() {
        return targetTime == -1;
    }

    @Override
    public void updateTimer() {
        if (System.currentTimeMillis() > targetTime && targetTime != -1) {
            setPower(0);
            targetTime = -1;
        }//if
    }

}
