package org.darbots.darbotsftclib.libcore.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.darbotsftclib.libcore.integratedfunctions.RobotLogger;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
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
        GlobalUtil.addLog("DarbotsBasicOpMode","Status","OpMode initialized", RobotLogger.LogLevel.DEBUG);
        this.waitForStart();
        if(this.opModeIsActive()){
            RunThisOpMode();
        }
        GlobalUtil.addLog("DarbotsBasicOpMode","Status","OpMode stopping", RobotLogger.LogLevel.DEBUG);
        this.getRobotCore().stop();
        GlobalUtil.addLog("DarbotsBasicOpMode","Status","OpMode finished", RobotLogger.LogLevel.DEBUG);
        this.getRobotCore().getLogger().saveLogsToFile();
        this.hardwareDestroy();
        GlobalRegister.runningOpMode = null;
    }
    @Override
    public void waitForStart(){
        while ((!opModeIsActive()) && (!isStopRequested())) {
            telemetry.addData("status", "Initialized, waiting for start command...");
            telemetry.update();
        }
        if(opModeIsActive()){
            telemetry.addData("status","Started");
            telemetry.update();
        }
    }

    public void waitForStart_NoDisplay(){
        super.waitForStart();
    }

    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.getRobotCore().updateStatus();
        }
        return this.opModeIsActive();
    }
}
