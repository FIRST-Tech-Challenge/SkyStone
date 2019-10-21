package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import org.firstinspires.ftc.teamcode.RC;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Windows on 2016-03-26.
 */
public class TOFSensor {

    public DigitalChannel channel;
    boolean currentlyTiming = false;
    long startNanoTime = -1;
    long latestTime = -2;
    boolean waitingForSignal = true;

    public TOFSensor(String name) {
        channel = RC.h.digitalChannel.get(name);
        channel.setMode(DigitalChannelController.Mode.INPUT);

    }//constructor

    public void check() {
        if (waitingForSignal) {
            boolean state = channel.getState();

            if (state && !currentlyTiming) {
                startNanoTime = System.nanoTime();
                currentlyTiming = true;
            } else if (!state && currentlyTiming) {
                latestTime = System.nanoTime() - startNanoTime;
                currentlyTiming = false;
                waitingForSignal = false;
            }//elseif
        }//if
    }//run

    public double getTimeOfFlight() {
        return latestTime;
    }//returnValue

}//TOFSensor
