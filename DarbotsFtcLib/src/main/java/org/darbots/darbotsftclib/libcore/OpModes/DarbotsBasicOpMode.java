package org.darbots.darbotsftclib.libcore.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

public abstract class DarbotsBasicOpMode<CoreType extends RobotCore> extends LinearOpMode {
    public abstract CoreType getRobotCore();
    public abstract void hardwareInitialize();
    public abstract void hardwareDestroy();
    public abstract void RunThisOpMode();
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalRegister.runningOpMode = this;
        this.hardwareInitialize();
        this.getRobotCore().getLogger().addLog("DarbotsBasicOpMode","Status","OpMode initialized");
        this.waitForStart();
        if(this.opModeIsActive()){
            RunThisOpMode();
        }
        this.getRobotCore().getLogger().addLog("DarbotsBasicOpMode","Status","OpMode stopping");
        this.getRobotCore().stop();
        this.getRobotCore().getLogger().addLog("DarbotsBasicOpMode","Status","OpMode finished");
        this.getRobotCore().getLogger().saveLogsToFile();
        this.hardwareDestroy();
        GlobalRegister.runningOpMode = null;
    }
}
