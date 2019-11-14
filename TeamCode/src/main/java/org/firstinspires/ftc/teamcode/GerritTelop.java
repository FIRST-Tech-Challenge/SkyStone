package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GerritTelop")
public class GerritTelop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    int position;
    float now;
    boolean a;
    public void leverArmStay() throws InterruptedException {
        if (a = true) {
            a = false;
            sleep(10);
            now = robot.leverArm.getCurrentPosition();
        }
        if (a = false){
            a = true;

        }
    }
    public void leverArmStay2() throws InterruptedException {
        position = robot.leverArm.getCurrentPosition();
        telemetry.addData("position%.2d", position);
            if (position > now) {
                robot.leverArm.setPower(-.4);
            }
            if (position < now) {
                robot.leverArm.setPower(.4);
            }
    }
    public void moveLeverArm(double distance) throws InterruptedException {
        telemetry.addData("Value %.2d", distance);
        position = robot.leverArm.getCurrentPosition() + 1;
        telemetry.addData("position%.2d", position);
        telemetry.update();

        if (distance > .1) {
            if (position >= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(0);
            }
            if (position >= 900) {
                robot.leverArm.setPower(-.27);
            }
            if (position <= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(.35);
            }
        }
        if (distance < -.1) {
            if (position <= 100) {
                robot.leverArm.setPower(0);
            }
            if (position <= 900) {
                robot.leverArm.setPower(.27);
            }
            if (position >= 100) {
                robot.leverArm.setPower(-.35);
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        a = true;
        waitForStart();
        int counter = 0;
        while (opModeIsActive()) {
            if (gamepad2.a) {
                leverArmStay();
            }
            if (a=false){
                leverArmStay2();
            }
            if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                moveLeverArm(-gamepad2.left_stick_y);
            }
            else {
                robot.leverArm.setPower(0);
            }
        }
    }
}