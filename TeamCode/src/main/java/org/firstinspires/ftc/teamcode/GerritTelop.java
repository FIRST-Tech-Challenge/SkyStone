package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GerritTelop")
public class GerritTelop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    int position;
    int wanted;

    public void leverArmStay() throws InterruptedException {
        position = robot.leverArm.getCurrentPosition();
        if (position > wanted) {
                robot.leverArm.setPower(-.4);
        }
        if (position < wanted) {
                robot.leverArm.setPower(.4);
        }
    }

    public void moveLeverArm(double distance) throws InterruptedException {
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
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        int counter = 0;

        while (opModeIsActive()) {

            if (gamepad2.left_stick_y < .5 && gamepad2.left_stick_y > -.5) {
                leverArmStay();
            }
            if (gamepad2.left_stick_y > .5 || gamepad2.left_stick_y < -.5) {
                moveLeverArm(-gamepad2.left_stick_y);
            }
        }
    }
}