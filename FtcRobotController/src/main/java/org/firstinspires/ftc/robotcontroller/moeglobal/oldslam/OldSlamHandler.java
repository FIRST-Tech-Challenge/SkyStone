package org.firstinspires.ftc.robotcontroller.moeglobal.oldslam;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class OldSlamHandler {

    private static OldUSBController usbController;

    public static void init(FtcRobotControllerActivity activity) {
        initUsbController(activity);
        initDevice();
    }

    private static void initDevice() {
        usbController.loadT265fw();
        usbController.openStream();
    }

    private static void initUsbController(FtcRobotControllerActivity activity) {
        usbController = new OldUSBController(activity, 0);
    }

    private static void registerListener() {

    }
}
