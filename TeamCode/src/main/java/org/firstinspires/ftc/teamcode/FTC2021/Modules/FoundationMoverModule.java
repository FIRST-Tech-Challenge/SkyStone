package org.firstinspires.ftc.teamcode.FTC2021.Modules;

import org.firstinspires.ftc.teamcode.FTC2021.HardwareCollection;
import org.firstinspires.ftc.teamcode.FTC2021.Robot;

import static org.firstinspires.ftc.teamcode.FTC2021.Constants.LEFTFOUNDATION_EXTENDED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.LEFTFOUNDATION_RETRACTED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.RIGHTFOUNDATION_EXTENDED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.RIGHTFOUNDATION_RETRACTED;

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
