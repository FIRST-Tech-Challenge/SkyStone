package org.firstinspires.ftc.teamcode.FTC2021.Modules;

import org.firstinspires.ftc.teamcode.FTC2021.HardwareCollection;
import org.firstinspires.ftc.teamcode.FTC2021.Robot;

import static org.firstinspires.ftc.teamcode.FTC2021.Constants.OUTTAKE_SLIDE_EXTENDED;
import static org.firstinspires.ftc.teamcode.FTC2021.Constants.OUTTAKE_SLIDE_RETRACTED;

public class ClawModule {

    public boolean extended;

    public StringBuilder clawData;

    public ClawModule(){
        extended = false;

        clawData = new StringBuilder();
        clawData.append("exended");
        clawData.append("\n");
    }

    public void update(Robot robot, HardwareCollection hardwareCollection) {
        if (robot.isDebug){
            clawData.append(extended);
            clawData.append("\n");
        }

        if (extended){
            hardwareCollection.outtakeExtender.setPosition(OUTTAKE_SLIDE_EXTENDED);
        } else {
            hardwareCollection.outtakeExtender.setPosition(OUTTAKE_SLIDE_RETRACTED);
        }
    }

}
