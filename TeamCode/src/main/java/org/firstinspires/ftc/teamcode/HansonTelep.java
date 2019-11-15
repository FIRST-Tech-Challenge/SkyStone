package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name = "HansonTelep")
public class HansonTelep extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    public void tankDrive(double leftStick, double rightStick) {


        if (leftStick > .2 || leftStick < -.2) {
            robot.leftDrive.setPower(leftStick);
        }
        if (rightStick > .2 || rightStick < -.2) {
            robot.rightDrive.setPower(rightStick);
        }
        if (rightStick < .2 && rightStick > -.2) {
            robot.rightDrive.setPower(0);
        }
        if (leftStick < .2 && leftStick > -.2) {
            robot.leftDrive.setPower(0);
        }
    }
    @Override
    public void runOpMode() {

        // Initialize, wait for start
        robot.init(hardwareMap, telemetry);
        waitForStart();

        // Begins while loop, updates telemetry
        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");
            telemetry.update();

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

            }
        }
    }
}