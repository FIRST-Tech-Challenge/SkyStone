package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem.Direction;
import org.firstinspires.ftc.teamcode.components.SpinnySystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "Drive", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    public void loop(){
        float rx = gamepad1.right_stick_x;
        float lx = gamepad1.left_stick_x;
        float ly = gamepad1.left_stick_y;

        if (gamepad1.a) {
            driveSystem.driveToPosition(1000, DriveSystem.Direction.RIGHT, 0.5);
        }
        if (gamepad1.b) {
            driveSystem.driveToPosition(1000, DriveSystem.Direction.LEFT, 0.5);
        }
        if (gamepad1.x) {
            driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 0.5);
        }
        if (gamepad1.y) {
            driveSystem.driveToPosition(1000, DriveSystem.Direction.BACKWARD, 0.5);
        }
        driveSystem.drive(rx, lx, ly);

        spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
    }
}