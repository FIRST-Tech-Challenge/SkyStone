package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "TestTeleop", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    public void loop(){
            driveSystem.drive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);
            spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
            latchSystem.run(gamepad1.dpad_up, gamepad1.dpad_down);
    }
}