package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class DriveTrain {

    public void drive_train(RobotHardware robot, double leftStick, double rightStick) {

        if (leftStick > .4 || leftStick < -.4) {
            robot.leftDrive.setPower(leftStick/2);
        }
        if (rightStick > .4 || rightStick < -.4) {
            robot.rightDrive.setPower(rightStick/2);
        }
        if (rightStick < .4 && rightStick > -.4) {
            robot.rightDrive.setPower(0);
        }
        if (leftStick < .4 && leftStick > -.4) {
            robot.leftDrive.setPower(0);
        }
    }
}
