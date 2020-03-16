package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class OuttakeModule {
    public OuttakeStateModule outtakeStateModule;
    public LinkageModule linkageModule;
    public ClawModule clawModule;

    public OuttakeModule(){
        outtakeStateModule = new OuttakeStateModule();
        linkageModule = new LinkageModule();
        clawModule = new ClawModule();
    }

    public void update(Robot robot, HardwareCollection hardwareCollection){
        outtakeStateModule.update(this);
        linkageModule.update(robot, hardwareCollection);
        clawModule.update(robot, hardwareCollection);
    }
}