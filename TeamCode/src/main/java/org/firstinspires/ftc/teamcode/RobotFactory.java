package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotFactory {
    static RobotHardware  theRobot = null;

    static public RobotHardware getRobotHardware(HardwareMap hardwareMap,RobotProfile robotProfile){
        if(theRobot == null){
            theRobot = new RobotHardware();
            theRobot.init(hardwareMap, robotProfile);
            theRobot.rotateGrabberOriginPos();
            theRobot.setClampPosition(RobotHardware.ClampPosition.INITIAL);
            theRobot.setHookPosition(RobotHardware.HookPosition.HOOK_OFF);
            theRobot.setCapStoneServo(RobotHardware.CapPosition.CAP_UP);

        }
        return theRobot;
    }

    public static void reset() {
        theRobot = null;
    }
}
