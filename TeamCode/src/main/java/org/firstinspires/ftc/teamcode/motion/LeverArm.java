package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class LeverArm {

    int position;
    int wanted;

    public void leverArmStay(RobotHardware robot){
        position = robot.leverArm.getCurrentPosition();
        if (position > wanted) {
            robot.leverArm.setPower(-.4);
        }
        if (position < wanted) {
            robot.leverArm.setPower(.4);
        }
    }

    public void moveLeverArm(RobotHardware robot, Telemetry telemetry, double distance){
        telemetry.addData("Value %.2d", distance);
        position = robot.leverArm.getCurrentPosition();
        telemetry.addData("position%.2d", position);
        telemetry.update();

        if (distance > .5) {
            if (position >= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(0);
            }
            else if (position >= 900) {
                robot.leverArm.setPower(-.02);
            }
            else if (position <= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(.35);
            }
        }
        if (distance < -.5) {
            if (position <= 100) {
                robot.leverArm.setPower(0);
            }
            else if (position <= 700) {
                robot.leverArm.setPower(.02);
            }
            else if (position >= 100) {
                robot.leverArm.setPower(-.35);
            }
        }
        wanted = robot.leverArm.getCurrentPosition();
    }

}
