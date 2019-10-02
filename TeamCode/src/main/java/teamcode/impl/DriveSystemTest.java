package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;

@Autonomous(name = "TT Drive System Test")
public class DriveSystemTest extends TTOpMode {

    @Override
    protected void onInitialize() {

    }

    @Override
    protected void onStart() {
        TTRobot robot = getRobot();
        TTDriveSystem driveSystem = robot.getHardwareManager().getDriveSystem();
        driveSystem.diagonal(1, 100, 0.5);
    }

}
