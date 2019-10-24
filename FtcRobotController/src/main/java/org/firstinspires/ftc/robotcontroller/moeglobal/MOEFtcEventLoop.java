package org.firstinspires.ftc.robotcontroller.moeglobal;

import android.app.Activity;
import android.util.Log;
import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.ProgrammingModeController;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.Command;
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase;
import org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading.OpModeLoading;
import org.firstinspires.ftc.robotcore.internal.network.CallbackResult;

public class MOEFtcEventLoop extends FtcEventLoop {


    public MOEFtcEventLoop(HardwareFactory hardwareFactory, OpModeRegister userOpmodeRegister, UpdateUI.Callback callback, Activity activityContext, ProgrammingModeController programmingModeController) {
        super(hardwareFactory, userOpmodeRegister, callback, activityContext, programmingModeController);

    }


    @Override
    public void init(EventLoopManager eventLoopManager) throws RobotCoreException, InterruptedException {
        super.init(eventLoopManager);
        MOEDatabase.codeStatusSync.setValue(false);
        OpModeLoading.loadOpModes();
    }

    @Override
    public CallbackResult processCommand(Command command) throws InterruptedException, RobotCoreException {
        String name = command.getName();
        if (name.equals(CommandList.CMD_INIT_OP_MODE) || name.equals(CommandList.CMD_RUN_OP_MODE)) {
            Log.e("command", command.toString());
        }
        return super.processCommand(command);
    }

    @Override
    protected void handleCommandRestartRobot() {
        MOEDatabase.codeStatusSync.setValue(false);
        super.handleCommandRestartRobot();
    }

    public void refreshUI() {
        this.sendUIState();
    }
}
