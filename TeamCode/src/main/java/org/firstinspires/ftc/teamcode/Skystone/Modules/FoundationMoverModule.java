package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import static org.firstinspires.ftc.teamcode.Skystone.Constants.*;

@Deprecated
public class FoundationMoverModule {

    public boolean isExtend;
    public StringBuilder foundationMoverData;

    public FoundationMoverModule(){
        isExtend = false;
        foundationMoverData = new StringBuilder();
        foundationMoverData.append("isExtend");
        foundationMoverData.append("\n");
    }

    public synchronized void update(Robot robot, HardwareCollection hardwareCollection){

        if (robot.isDebug){
            foundationMoverData.append(isExtend);
            foundationMoverData.append("\n");
        }

        if (isExtend) {
            hardwareCollection.leftFoundation.setPosition(LEFTFOUNDATION_EXTENDED);
            hardwareCollection.rightFoundation.setPosition(RIGHTFOUNDATION_EXTENDED);
        } else {
            hardwareCollection.leftFoundation.setPosition(LEFTFOUNDATION_RETRACTED);
            hardwareCollection.rightFoundation.setPosition(RIGHTFOUNDATION_RETRACTED);
        }
    }
}
