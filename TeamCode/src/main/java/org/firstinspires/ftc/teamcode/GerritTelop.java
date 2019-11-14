package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GerritTelop")
public class GerritTelop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    int position;
    float game;

    public void moveLeverArm(double distance) throws InterruptedException {
        telemetry.addData("Value %.2d", distance);
        position = robot.leverArm.getCurrentPosition() + 1;
        telemetry.addData("position%.2d", position);
        telemetry.update();
        if (distance > .1) {
            telemetry.addLine("Working11");
            telemetry.update();
            if (position <= robot.ARM_UP_DISTANCE) {
                telemetry.addLine("Working12");
                telemetry.update();
                robot.leverArm.setPower(1);
            }
            if (position >= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(0);
            }
        }
        if (distance < -.1) {
            telemetry.addLine("Working21");
            telemetry.update();
            if (position <= 0) {
                robot.leverArm.setPower(0);
            }
            if (position >= 0) {
                telemetry.addLine("Working22");
                telemetry.update();
                robot.leverArm.setPower(-1);
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        int counter = 0;
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {
                moveLeverArm(-gamepad1.left_stick_y);
            }
            else {
                robot.leverArm.setPower(0);
            }
        }
    }
}