package org.firstinspires.ftc.teamcode.pidBullshit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Maccabot;

@TeleOp(name="Why da hecc does i do this")
//@Disabled
public class PIDTuningOpMode extends OpMode {

    Maccabot robot;

    @Override
    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
    }

    @Override
    public void loop() {
        robot.drive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
