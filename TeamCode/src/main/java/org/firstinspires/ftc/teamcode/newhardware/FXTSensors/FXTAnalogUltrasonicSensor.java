package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.robots.Fermion;

/**
 * Created by Windows on 2016-12-30.
 */

public class FXTAnalogUltrasonicSensor {
    AnalogInput ultra;
    DigitalChannel range;

    public FXTAnalogUltrasonicSensor(String name, String range){
        ultra = RC.h.analogInput.get(name);
        this.range = RC.h.digitalChannel.get(range);
        this.range.setMode(DigitalChannelController.Mode.OUTPUT);
        this.range.setState(true);
    }

    public FXTAnalogUltrasonicSensor(String name){
        ultra = RC.h.analogInput.get(name);
    }

    public double getDistance(){
        return ultra.getVoltage() * 1766 / ultra.getMaxVoltage();
    }
}
//added 2 zeros