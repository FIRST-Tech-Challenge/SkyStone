package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import static org.firstinspires.ftc.teamcode.Skystone.Constants.*;

public class ClawModule {

    public boolean extended;

    public ClawModule(){
        extended = false;
    }

    public void update(Robot robot, HardwareCollection hardwareCollection) {
        if (extended){
            hardwareCollection.outtakeExtender.setPosition(OUTTAKE_SLIDE_EXTENDED);
        } else {
            hardwareCollection.outtakeExtender.setPosition(OUTTAKE_SLIDE_RETRACTED);
        }
    }

}
