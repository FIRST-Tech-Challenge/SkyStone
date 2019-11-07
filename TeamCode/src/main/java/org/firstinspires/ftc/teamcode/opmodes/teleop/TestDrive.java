package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "ControllerTest", group="TeleOp")
public class TestDrive extends OpMode {
    public final String TAG = "TESTTEST";

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        /*
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);
        driveSystem.drive(rx, lx, -ly, gamepad1.x);
        */
        Log.d(TAG, "gamepad: " + gamepad1.toString());
       /* Log.d(TAG, "right trig: " + gamepad1.right_trigger);
        Log.d(TAG, "left bump: " + gamepad1.left_bumper);
        Log.d(TAG, "right bump: " + gamepad1.right_bumper);

        */
    }
}
