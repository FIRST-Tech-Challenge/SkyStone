package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Control.HoldingPIDMotor;

@Config
public class Extender {
    public static int MAX_EXTENDER_POS = 1250;
    public static int MIN_EXTENDER_POS = 0;
    public static int COLLECT_POS = 0;

    public static int MAXED_THRESHOLD = 900;
    public static int MIN_THRESHOLD = 300;
    public static int RESET_TRUE_POSITION = 1175;
    public static int CUT_POWER_MS = 200;
    public static double factor = 0.000001;

    public static int SEEK_THRESHOLD = 50;

    private DigitalChannel slideSwitch;
    private HoldingPIDMotor extender;
    private ElapsedTime timeAtMaxima;
    private boolean prevState;
    private int offset;

    public Extender(DigitalChannel slideSwitch, DcMotorEx extender) {
        this.extender = new HoldingPIDMotor(extender, 1.0);
        this.slideSwitch = slideSwitch;
        this.timeAtMaxima = new ElapsedTime();
    }

    public void setPower(double p) {
        if (maxExtend() && p > 0 || minExtend() && p < 0) {
            if (timeAtMaxima.milliseconds() > CUT_POWER_MS) {
                extender.setPower(p * factor);
            } else {
                extender.setPower(p);
            }
        } else {
            extender.setPower(p);
            timeAtMaxima.reset();
        }
        boolean currState = slideSwitch.getState();
        if (prevState && !currState) {
            // Reset encoder
            offset = extender.getCurrentPosition() - RESET_TRUE_POSITION;
            prevState = false;
        } else if (currState && !prevState) {
            prevState = true;
        }
    }

    public void goToPosition(int position) {
        extender.setTargetPos(position + offset);
    }

    public int getPosition() {
        return extender.getCurrentPosition() - offset;
    }

    public boolean maxExtend() { return getPosition() > MAXED_THRESHOLD; }
    public boolean minExtend() { return getPosition() < MIN_THRESHOLD; }
    public void goToMax() { goToPosition(MAX_EXTENDER_POS); }
    public void goToMin() { goToPosition(MIN_EXTENDER_POS); }
    public void goToCollect() { goToPosition(COLLECT_POS); }

    public boolean seeking() {
        return extender.getErr() < SEEK_THRESHOLD;
    }
}
