package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.DriveSystem.Direction;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "TestTeleop", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    public void loop(){
            float rx = gamepad1.right_stick_x;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;

            if (gamepad1.a) {
                driveSystem.driveToPosition(1000, Direction.RIGHT, 0.5);
            }
            if (gamepad1.b) {
                driveSystem.driveToPosition(1000, Direction.LEFT, 0.5);
            }
            if (gamepad1.x) {
                driveSystem.driveToPosition(1000, Direction.FORWARD, 0.5);
            }
            if (gamepad1.y) {
                driveSystem.driveToPosition(1000, Direction.BACKWARD, 0.5);
            }
            driveSystem.drive(rx, lx, ly);
            spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
            latchSystem.run(gamepad1.dpad_up, gamepad1.dpad_down);

    }
}