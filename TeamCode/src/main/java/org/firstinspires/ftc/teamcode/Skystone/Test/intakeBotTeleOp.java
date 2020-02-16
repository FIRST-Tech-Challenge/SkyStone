package org.firstinspires.ftc.teamcode.Skystone.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

@TeleOp
public class intakeBotTeleOp extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();

        telemetry.addLine("thinking inside the sphere since today");
        telemetry.update();

        while (opModeIsActive()) {
            driveLogic();
            intakeLogic();
        }
    }

    private void initLogic() {
        robot = new Robot(hardwareMap, telemetry, this);
    }

    private void driveLogic() {
        double fLeftPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
        double fRightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
        double bLeftPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
        double bRightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

        robot.allWheelDrive(fLeftPower, fRightPower, bLeftPower, bRightPower);
    }

    private void intakeLogic() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        robot.getIntakeRight().setPower(intakePower);
        robot.getIntakeLeft().setPower(-intakePower);
    }
}
