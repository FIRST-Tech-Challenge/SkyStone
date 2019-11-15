package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.movement.MecanumDrive;

public class IterativeOpMode extends OpMode {
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = MecanumDrive.standard(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
    }
}
