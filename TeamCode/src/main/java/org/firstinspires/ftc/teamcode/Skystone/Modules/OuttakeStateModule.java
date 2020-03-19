package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class OuttakeStateModule {

    public int state;
    public StringBuilder outtakeStateData;

    public OuttakeStateModule(){
        state = 0;
        outtakeStateData = new StringBuilder();
        outtakeStateData.append("state");
        outtakeStateData.append("\n");
    }

    public void update(Robot robot, HardwareCollection hardwareCollection) {
        if (robot.isDebug){
            outtakeStateData.append(state);
            outtakeStateData.append("\n");
        }
    }

}
