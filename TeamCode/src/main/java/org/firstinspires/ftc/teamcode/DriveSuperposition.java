package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Superposition Drive")
public class DriveSuperposition extends DriveHalo {

    @Override
    void driveController() {
        if (gamepad1.right_bumper) {
            speedControl = 0.15;
        } else {
            speedControl = 0.5;
        }

        // Joysticks
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        // Convert (lx, ly) to polar
        double r = Math.hypot(lx, ly);
        double theta = Math.atan2(ly, lx) - Math.PI / 4;

        robot.frontLeft.setPower(speedControl * (r * Math.sin(theta) - rx));
        robot.frontRight.setPower(speedControl * (r * Math.cos(theta) + rx));
        robot.rearLeft.setPower(speedControl * (r * Math.cos(theta) - rx));
        robot.rearRight.setPower(speedControl * (r * Math.sin(theta) + rx));
    }
}
