package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@TeleOp (name = "Tank Drive")
public class DriveTank extends OpMode {

    private Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Initialized", "Ready to start");

    }

    @Override
    public void loop() {
        double speedControl = 0.5; // to make the robot go slower
        // activate slowMode if both joysticks are pushed down
        boolean slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button;

        if (slowMode) {
            speedControl = 0.25; // slowMode reduces the speed control
        }

        // Joystick
        double leftPower = speedControl * -gamepad1.left_stick_y;
        double rightPower = speedControl * -gamepad1.right_stick_y;
        double leftStrafe = speedControl * gamepad1.left_stick_x;
        double rightStrafe = speedControl * gamepad1.right_stick_x;

        // Strafe
        if (leftStrafe == rightStrafe && leftPower == 0 && rightPower == 0) {
            robot.setStrafe(leftPower);
        }

        // Not strafing
        else {
            robot.rearLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);
            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
    }
}