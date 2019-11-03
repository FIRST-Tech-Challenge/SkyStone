package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "TestTeleop", group="TeleOp")
public class TestTeleOp extends BaseOpMode {
    private boolean slowDrive;

    @Override
    public void init() {
        slowDrive = false;
    }

    public void loop(){
        if(gamepad1.left_stick_button) {
            if (slowDrive) {
                slowDrive = false;
            } else {
                slowDrive = true;
            }
        }
        driveSystem.drive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y, slowDrive);
        spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
        latchSystem.run(gamepad1.dpad_up, gamepad1.dpad_down);
    }


}