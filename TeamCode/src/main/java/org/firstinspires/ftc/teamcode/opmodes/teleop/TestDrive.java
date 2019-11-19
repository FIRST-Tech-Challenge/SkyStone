package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "TeleopTestDrive", group="TeleOp")
public class TestDrive extends BaseOpMode {
    public void init() {
        super.init();
    }
    public void loop() {
        float rx = (float) Math.pow(gamepad1.right_stick_x, 5);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 5);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 5);
        driveSystem.drive(rx, lx, -ly, gamepad1.left_trigger);
    }
}
