package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Windows on 2016-03-15.
 */
public class DigitalUltrasonicSensor extends TOFSensor {

    DigitalChannel trigger;
    boolean sendPulse = false;
    long pulseBeginTime = -2;


    public DigitalUltrasonicSensor(String echoName, String triggerName) {
        super(echoName);

        trigger = RC.h.digitalChannel.get(triggerName);
        trigger.setMode(DigitalChannelController.Mode.OUTPUT);
        waitingForSignal = false;
    }//constructor

    public void check() {
        long time = System.nanoTime();
        super.check();


        if (sendPulse) {
            trigger.setState(true);
            pulseBeginTime = System.nanoTime();
            sendPulse = false;
        } else if (pulseBeginTime != -2 && System.nanoTime() - pulseBeginTime > 5000) {
            trigger.setState(false);
            pulseBeginTime = -2;
        } else if (System.nanoTime() - pulseBeginTime < 50000000) {
            Log.i("still on", "h" + System.nanoTime());
        }
        Log.i("joo", "" + (System.nanoTime() - time));
    }//run

    public void pulse() {
        sendPulse = true;
        waitingForSignal = true;
    }//pulse

    public double returnValue() {
        return ((super.getTimeOfFlight()) * 340.29 / 1000000000) * 1000;
    }

}//DigitalUltrasonicSensor
