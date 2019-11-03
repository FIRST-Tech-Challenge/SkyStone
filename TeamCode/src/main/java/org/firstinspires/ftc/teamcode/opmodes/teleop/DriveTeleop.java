package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "DriveTeleOp", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    private boolean slowDrive;
    private static final String TAG = "DriveTeleOp";

    @Override
    public void init() {
        super.init();
        slowDrive = false;
    }

    public void loop(){
        driveSystem.drive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.x);
        spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
        latchSystem.run(gamepad1.dpad_up, gamepad1.dpad_down);
    }
}