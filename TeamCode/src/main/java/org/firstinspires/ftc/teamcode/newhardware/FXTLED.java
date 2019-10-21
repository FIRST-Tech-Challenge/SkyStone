package org.firstinspires.ftc.teamcode.newhardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;

/**
 * Created by Windows on 2017-01-19.
 */

public class FXTLED implements FXTDevice {
    DigitalChannel pin;

    public FXTLED(String name){
        pin = RC.h.digitalChannel.get(name);
        pin.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    public void illuminate(){
        pin.setState(true);
    }

    public void extinguish(){
        pin.setState(false);
    }

    public void switchState(){
        pin.setState(!pin.getState());
    }

    public void blink(int delay, int count){
        TaskHandler.addCountedTask("blinker", blinker(delay), count);
    }

    private Runnable blinker(final int delay){
        return new Runnable() {
            @Override
            public void run() {
                illuminate();
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                extinguish();
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };
    }


}
