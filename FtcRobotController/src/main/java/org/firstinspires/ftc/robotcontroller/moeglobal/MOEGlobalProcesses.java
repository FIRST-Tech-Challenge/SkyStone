package org.firstinspires.ftc.robotcontroller.moeglobal;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEFirebase;
import org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading.OpModeLoading;
import org.firstinspires.ftc.robotcontroller.moeglobal.slam.SlamHandler;

public class MOEGlobalProcesses {


    public static void preInit(FtcRobotControllerActivity activity) {
        OpModeLoading.init(activity);
        MOEFirebase.init(activity);
        SlamHandler.init(activity);

    }

    public static void postInit(FtcRobotControllerActivity activity) {
    }

}
