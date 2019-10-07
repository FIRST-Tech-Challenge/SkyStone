package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;

@Autonomous(name = "Vision Test")
public class VisionTest extends TTOpMode {

    @Override
    protected void onInitialize() {
        TTRobot robot = getRobot();
        robot.getVision().activate();
    }

    @Override
    protected void onStart() {
    }

}
