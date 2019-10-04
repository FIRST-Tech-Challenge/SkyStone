package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;

@Autonomous(name = "TT Drive System Test")
public class DriveSystemTest extends TTOpMode {

    @Override
    protected void onInitialize() {
        TTRobot robot = getRobot();
        robot.getVision().init();
    }

    @Override
    protected void onStart() {
    }

}
