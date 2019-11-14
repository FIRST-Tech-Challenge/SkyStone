package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Clamp {

    public void setClamp(RobotHardware robot, boolean open, boolean close) {
        if (open) {
            robot.clamp.setPosition(robot.CLAMP_OPEN_DISTANCE);
        }

        if (close) {
            robot.clamp.setPosition(robot.CLAMP_CLOSE_DISTANCE);
        }
    }

    public void moveClampRotator(RobotHardware robot, double clamp_rotator_set) {
        if (clamp_rotator_set > 0) {
            robot.clampRotator.setPosition(.002 + robot.clampRotator.getPosition());
        } else {
            robot.clampRotator.setPosition(-.002 + robot.clampRotator.getPosition());
        }

    }
}
