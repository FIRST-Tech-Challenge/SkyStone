package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;

import static org.firstinspires.ftc.teamcode.Skystone.Constants.*;

public class FoundationMoverModule {

    public boolean isExtend;

    public FoundationMoverModule(){
        isExtend = false;
    }

    public synchronized void update(HardwareCollection hardwareCollection){
        if (isExtend) {
            hardwareCollection.leftFoundation.setPosition(LEFTFOUNDATION_EXTENDED);
            hardwareCollection.rightFoundation.setPosition(RIGHTFOUNDATION_EXTENDED);
        } else {
            hardwareCollection.leftFoundation.setPosition(LEFTFOUNDATION_RETRACTED);
            hardwareCollection.rightFoundation.setPosition(RIGHTFOUNDATION_RETRACTED);
        }
    }
}
