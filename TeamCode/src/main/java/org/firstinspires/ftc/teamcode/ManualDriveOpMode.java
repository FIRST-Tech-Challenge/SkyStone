package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Mecanum")
public class ManualDriveOpMode extends LinearOpMode {
    private ScoopArmBot robot;
    private boolean arcadeMode = false;
    private int gyroCalibratedCount = 0;



    @Override
    public void runOpMode() {
        robot = new ScoopArmBot(this);
        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.resetHeading();
            }
            robot.driveByHand(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, arcadeMode);
            robot.scoopSetPosition(gamepad1.b, gamepad1.y, gamepad1.x);
            robot.scoopFreeRun(gamepad1.right_bumper, gamepad1.left_bumper);
        }
    }
}

